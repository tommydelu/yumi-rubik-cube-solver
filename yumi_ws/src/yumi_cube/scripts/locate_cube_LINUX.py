#!/usr/bin/env python3
"""
Nodo di percezione del cubo, versione 100% ROS2-nativa: nessuna connessione
Remote API, nessun parametro manuale da configurare. Tutto arriva da topic
pubblicati dallo script Lua su Vision_sensor2 via simROS2:

  - /yumi/vision_cube/depth        (sensor_msgs/msg/Image, encoding 32FC1)
  - /sensor2_pose                  (geometry_msgs/msg/Pose)
  - /yumi/vision_cube/camera_info  (sensor_msgs/msg/CameraInfo: fx, fy, cx, cy, risoluzione)
  - /yumi/vision_cube/far_clip     (std_msgs/msg/Float32)

Vedi vision_sensor2_publisher.lua per lo script Lua corrispondente.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Bool
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

CUBE_SIZE = 0.04
OUTPUT_DIR = '/home/delu/Desktop/vision_ai/yumi_ws'  # aggiornare al path locale (solo per vedere immagini point cloud finale)


class CubePerceptionNode(Node):
    def __init__(self):
        super().__init__('locate_cube_node_LINUX')

        # Stato ricevuto dai topic - nessun parametro manuale
        self.latest_depth = None          # np.ndarray (H, W) float32
        self.latest_sensor_pose = None    # (position[3], quaternion_xyzw[4])
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.far_clip = None
        self.detection_done = False
        self.ready_to_process = False

        self.pub_cube_pcl = self.create_publisher(Pose, '/get_cube_pose', 10)

        self.create_subscription(Image, '/yumi/vision_cube/depth', self.depth_cb, 10)
        self.create_subscription(Pose, '/sensor2_pose', self.sensor_pose_cb, 10)
        self.create_subscription(CameraInfo, '/yumi/vision_cube/camera_info', self.camera_info_cb, 10)
        self.create_subscription(Float32, '/yumi/vision_cube/far_clip', self.far_clip_cb, 10)
        self.create_subscription(Bool, '/locate_cube/ready', self.locate_ready_cb, 10)

        self.get_logger().info("In attesa di depth, posa, camera_info e far_clip sui topic...")

    def locate_ready_cb(self, msg: Bool):
        self.ready_to_process = msg.data
        if msg.data:
            self.detection_done = False
            self.get_logger().info("Richiesta ricevuta su /locate_cube/ready: calcolo posa cubo abilitato.")
            self.try_process()
        else:
            self.get_logger().info("Richiesta su /locate_cube/ready disattivata.")


    def camera_info_cb(self, msg: CameraInfo):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.try_process()

    def far_clip_cb(self, msg: Float32):
        self.far_clip = msg.data
        self.try_process()

    def sensor_pose_cb(self, msg: Pose):
        position = np.array([msg.position.x, msg.position.y, msg.position.z])
        quaternion = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.latest_sensor_pose = (position, quaternion)
        self.try_process()

    def depth_cb(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f"Encoding depth inatteso: {msg.encoding} (atteso 32FC1)")
            return
        depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape((msg.height, msg.width))
        self.latest_depth = depth
        self.try_process()

    def try_process(self):
        if not self.ready_to_process:
            return
        if self.detection_done:
            return
        if (self.latest_depth is None or self.latest_sensor_pose is None
                or self.fx is None or self.far_clip is None):
            return
        self.get_logger().info(
            f"Tutti i dati ricevuti (fx={self.fx:.1f}, fy={self.fy:.1f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f}, far_clip={self.far_clip:.2f}m). "
            f"Calcolo la Point Cloud..."
        )
        self.process_point_cloud_oneshot()

    def estimate_center_multi_face(self, cloud):
        remaining = cloud
        face_centroids = []
        face_normals = []

        for _ in range(3):
            if len(remaining.points) < 20:
                break
            plane_model, inliers = remaining.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=500)
            if len(inliers) < 15:
                break
            face_cloud = remaining.select_by_index(inliers)
            centroid = np.asarray(face_cloud.points).mean(axis=0)
            normal = np.array(plane_model[:3])
            normal = normal / np.linalg.norm(normal)

            direction_to_cam = -centroid
            if np.dot(normal, direction_to_cam) < 0:
                normal = -normal

            face_centroids.append(centroid)
            face_normals.append(normal)

            remaining = remaining.select_by_index(inliers, invert=True)
            if len(remaining.points) < 15:
                break

        if not face_centroids:
            return None, None

        self.get_logger().info(f"Facce rilevate per la stima del centro: {len(face_centroids)}")

        center_estimate = np.zeros(3)
        for c, n in zip(face_centroids, face_normals):
            center_estimate += c - n * (CUBE_SIZE / 2.0)
        center_estimate /= len(face_centroids)

        representative_normal = face_normals[0]
        return center_estimate, representative_normal

    def isolate_cube_cluster(self, cloud):
        pts = np.asarray(cloud.points)
        if len(pts) < 10:
            return None
        labels = np.array(cloud.cluster_dbscan(eps=CUBE_SIZE * 1.2, min_points=6, print_progress=False))
        if labels.max() < 0:
            return None
        best_idx, best_score = None, None
        for lbl in range(labels.max() + 1):
            idx = np.where(labels == lbl)[0]
            cluster_pts = pts[idx]
            extent = cluster_pts.max(axis=0) - cluster_pts.min(axis=0)
            max_extent = extent.max()
            self.get_logger().info(f"Cluster {lbl}: {len(idx)} punti, estensione max {max_extent*100:.2f} cm")
            if max_extent > CUBE_SIZE * 2.5:
                continue
            score = abs(max_extent - CUBE_SIZE)
            if best_score is None or score < best_score:
                best_score, best_idx = score, idx
        if best_idx is None:
            return None
        return cloud.select_by_index(best_idx.tolist())

    def process_point_cloud_oneshot(self):
        Z = self.latest_depth
        res_y, res_x = Z.shape
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        far_clip = self.far_clip

        u, v = np.meshgrid(np.arange(res_x), np.arange(res_y))
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
        valid_points = points[Z.flatten() < (far_clip - 0.01)]

        if len(valid_points) < 100:
            self.get_logger().warn("Troppo pochi punti validi nella depth ricevuta. Ritento al prossimo frame.")
            self.latest_depth = None
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(valid_points)

        # Conversione da convenzione pinhole (X destra, Y giù, Z avanti) a
        # convenzione locale del sensore in CoppeliaSim (X sinistra, Y su,
        # Z avanti): si inverte X e Y, Z resta invariato.
        pcd.transform([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        # --- RANSAC TAVOLO ---
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.004, ransac_n=3, num_iterations=1000)
        a, b, c, d = plane_model
        cube_cloud = pcd.select_by_index(inliers, invert=True)

        if len(cube_cloud.points) < 10:
            self.get_logger().warn("Nessun punto rimasto dopo la rimozione del tavolo.")
            return

        # --- FILTRO PER SALTO DI PROFONDITA' ---
        pts_remaining = np.asarray(cube_cloud.points)
        signed_dist = (a * pts_remaining[:, 0] + b * pts_remaining[:, 1] + c * pts_remaining[:, 2] + d)
        if np.median(signed_dist) < 0:
            keep_mask = signed_dist < -0.003
        else:
            keep_mask = signed_dist > 0.003
        idx_keep = np.where(keep_mask)[0]
        cube_cloud = cube_cloud.select_by_index(idx_keep.tolist())

        self.get_logger().info(f"Punti cubo dopo filtro profondità: {len(cube_cloud.points)}")

        if len(cube_cloud.points) < 10:
            self.get_logger().warn("Troppo pochi punti dopo il filtro di profondità.")
            return

        # --- ISOLAMENTO DEL CUBO PER DIMENSIONE (DBSCAN) ---
        isolated = self.isolate_cube_cluster(cube_cloud)
        if isolated is not None and len(isolated.points) >= 10:
            cube_cloud = isolated
        else:
            self.get_logger().warn("DBSCAN non ha isolato un cluster valido: uso tutti i punti rimasti dopo il filtro profondità.")

        pts_cube_final = np.asarray(cube_cloud.points)

        # --- STIMA CENTRO MULTI-FACCIA ---
        geometric_center_local, normal = self.estimate_center_multi_face(cube_cloud)
        if geometric_center_local is None:
            self.get_logger().warn("Impossibile rilevare facce nel cluster del cubo; uso centroide semplice come fallback.")
            centroid = pts_cube_final.mean(axis=0)
            fallback_normal = np.array([a, b, c])
            fallback_normal = fallback_normal / np.linalg.norm(fallback_normal)
            if np.median(signed_dist) < 0:
                fallback_normal = -fallback_normal
            geometric_center_local = centroid - fallback_normal * (CUBE_SIZE / 2.0)
            normal = fallback_normal

        # --- DEBUG: SALVATAGGIO DI 4 VISTE PNG ---
        try:
            all_pts = np.vstack((pts_cube_final, geometric_center_local.reshape(1, 3)))
            max_range = max(np.array([all_pts[:, 0].max() - all_pts[:, 0].min(),
                                       all_pts[:, 1].max() - all_pts[:, 1].min(),
                                       all_pts[:, 2].max() - all_pts[:, 2].min()]).max() / 2.0, 0.01)
            mid_x = (all_pts[:, 0].max() + all_pts[:, 0].min()) * 0.5
            mid_y = (all_pts[:, 1].max() + all_pts[:, 1].min()) * 0.5
            mid_z = (all_pts[:, 2].max() + all_pts[:, 2].min()) * 0.5

            viste = [
                (30, 45, "1_isometrica"),
                (0, 0, "2_lato_X"),
                (0, 90, "3_lato_Y"),
                (90, -90, "4_alto")
            ]

            for elev, azim, nome_vista in viste:
                fig = plt.figure(figsize=(8, 6))
                ax = fig.add_subplot(111, projection='3d')
                ax.scatter(pts_cube_final[:, 0], pts_cube_final[:, 1], pts_cube_final[:, 2], c='green', s=8, label='Punti Cubo')
                ax.scatter(geometric_center_local[0], geometric_center_local[1], geometric_center_local[2], c='red', s=100, marker='o', label='Centro Stimato')
                ax.set_xlim(mid_x - max_range, mid_x + max_range)
                ax.set_ylim(mid_y - max_range, mid_y + max_range)
                ax.set_zlim(mid_z - max_range, mid_z + max_range)
                ax.set_title(f'Debug Cubo - {nome_vista.replace("_", " ").title()}')
                ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                ax.view_init(elev=elev, azim=azim)
                plt.savefig(f'{OUTPUT_DIR}/debug_cubo_{nome_vista}.png')
                plt.close(fig)
            self.get_logger().info("Salvate 4 immagini di debug.")
        except Exception as e_img:
            self.get_logger().error(f"Impossibile salvare l'immagine di debug: {e_img}")

        # --- TRASFORMAZIONE LOCALE -> GLOBALE (world frame) ---
        sensor_position, sensor_quaternion = self.latest_sensor_pose

        rot_matrix_sensor = R.from_quat(sensor_quaternion).as_matrix()
        geometric_center_global = np.dot(rot_matrix_sensor, geometric_center_local) + sensor_position

        z_axis = normal
        temp_x = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x_axis = temp_x - np.dot(temp_x, z_axis) * z_axis
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        rot_matrix_local = np.column_stack((x_axis, y_axis, z_axis))
        quat_local = R.from_matrix(rot_matrix_local).as_quat()

        r_sensor = R.from_quat(sensor_quaternion)
        r_local = R.from_quat(quat_local)
        quat_global = (r_sensor * r_local).as_quat()

        msg = Pose()
        msg.position.x = float(geometric_center_global[0])
        msg.position.y = float(geometric_center_global[1])
        msg.position.z = float(geometric_center_global[2])
        msg.orientation.x = float(quat_global[0])
        msg.orientation.y = float(quat_global[1])
        msg.orientation.z = float(quat_global[2])
        msg.orientation.w = float(quat_global[3])

        self.pub_cube_pcl.publish(msg)
        self.detection_done = True
        self.ready_to_process = False
        self.get_logger().info(f"Posa cubo pubblicata su /get_cube_pose! X:{msg.position.x:.3f} Y:{msg.position.y:.3f} Z:{msg.position.z:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = CubePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()