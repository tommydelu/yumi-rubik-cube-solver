import numpy as np
import open3d as o3d
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- 1. CONFIGURAZIONE INIZIALE ---
# Dimensioni note del cubo (in metri)
CUBE_SIZE = 0.05 

# Connessione a CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')

# Ottieni l'handle del sensore di visione (sostituisci con il nome esatto nella tua scena)
sensor_handle = sim.getObject('/Vision_sensor')

def get_point_cloud_from_sensor(sensor_handle):
    """Estrae la mappa di profondità e la converte in una Point Cloud 3D locale."""
    # Ottieni la risoluzione e i parametri della camera
    res_x, res_y = sim.getVisionSensorResolution(sensor_handle)
    angle = sim.getObjectFloatParam(sensor_handle, sim.visionfloatparam_perspective_angle)
    far_clip = sim.getObjectFloatParam(sensor_handle, sim.visionfloatparam_far_clipping)
    
    # 1. Leggi il buffer di profondità (opzione 1 = in METRI). 
    # Restituisce una tupla (dati_binari, risoluzione)
    depth_data, _ = sim.getVisionSensorDepth(sensor_handle, 1, [0, 0], [res_x, res_y])
    
    # 2. Decodifica i bytes in un array numpy di float32
    depth_img = np.frombuffer(depth_data, dtype=np.float32).reshape((res_y, res_x))
    
    # Capovolgi l'immagine sull'asse Y (CoppeliaSim legge dal basso verso l'alto)
    # Avendo usato l'opzione '1', depth_img è GIÀ la nostra coordinata Z in metri
    Z = np.flipud(depth_img)
    
    # Calcola i parametri intrinseci per convertire i pixel in coordinate 3D X e Y
    cx, cy = res_x / 2.0, res_y / 2.0
    fx = cx / np.tan(angle / 2.0)
    fy = cy / np.tan(angle / 2.0)
    
    u, v = np.meshgrid(np.arange(res_x), np.arange(res_y))
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    
    # Crea un array di punti (N, 3) e rimuovi i punti di background (Z troppo lontana)
    points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
    valid_points = points[Z.flatten() < (far_clip - 0.01)] 
    
    # Crea l'oggetto Point Cloud di Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(valid_points)
    
    # Raddrizza il sistema di coordinate per allineare Open3D a CoppeliaSim
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    return pcd

def main():
    print("Estrazione Point Cloud...")
    pcd = get_point_cloud_from_sensor(sensor_handle)
    
    # --- 2. RIMOZIONE DEL TAVOLO (RANSAC) ---
    # Cerchiamo il piano più grande nella scena (che presumibilmente è il tavolo)
    # distance_threshold = tolleranza in metri per considerare un punto appartenente al piano
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    
    print(f"Equazione del piano del tavolo: {plane_model[0]:.4f}x + {plane_model[1]:.4f}y + {plane_model[2]:.4f}z + {plane_model[3]:.4f} = 0")
    
    # Separiamo il tavolo dal resto degli oggetti
    table_cloud = pcd.select_by_index(inliers)
    cube_cloud = pcd.select_by_index(inliers, invert=True)
    
    # Puliamo la point cloud del cubo da eventuale rumore statistico (outliers)
    cube_cloud, ind = cube_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    if len(cube_cloud.points) < 50:
        print("Errore: Non ci sono abbastanza punti per identificare il cubo. Controlla il sensore.")
        return

    # --- 3. FIT DELLE FACCE DEL CUBO ---
    # RANSAC sulla point cloud rimanente per trovare la faccia principale esposta al sensore
    face_model, face_inliers = cube_cloud.segment_plane(distance_threshold=0.005,
                                                        ransac_n=3,
                                                        num_iterations=1000)
    
    face_cloud = cube_cloud.select_by_index(face_inliers)
    
    # La normale al piano estratto ci dà l'orientamento della faccia
    normal = np.array(face_model[:3])
    # Assicuriamoci che la normale punti verso la telecamera
    if normal[2] > 0:
        normal = -normal

    # --- 4. CALCOLO DEL CENTRO GEOMETRICO ---
    # Troviamo il baricentro (centroide) dei punti che compongono la faccia
    face_centroid = np.asarray(face_cloud.points).mean(axis=0)
    
    # Sapendo quanto è grande il cubo (CUBE_SIZE), il suo centro geometrico si trova 
    # traslando il baricentro della faccia lungo la normale al piano di metà della sua lunghezza
    geometric_center_local = face_centroid - normal * (CUBE_SIZE / 2.0)
    
    print("--- RISULTATI ---")
    print(f"Centroide della faccia vista: {face_centroid}")
    print(f"Centro Geometrico stimato del cubo: {geometric_center_local}")

   # --- 5. VISUALIZZAZIONE FINALE ---
    print("Apertura visualizzatore 3D... (Premi 'Q' nella finestra per chiudere)")
    
    # 1. Colora il resto del cubo (esclusa la faccia superiore) di grigio chiaro
    cube_cloud.paint_uniform_color([0.7, 0.7, 0.7])
    
    # 2. Colora la faccia fittata di verde acceso
    face_cloud.paint_uniform_color([0.0, 1.0, 0.0])
    
    # 3. Crea una piccola sfera 3D per visualizzare il centro geometrico stimato
    # Usiamo un raggio molto piccolo (es. 2 millimetri)
    center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.002)
    center_sphere.translate(geometric_center_local)
    center_sphere.paint_uniform_color([1.0, 0.0, 0.0]) # Sfera rossa per il centro
    
    # 4. (Opzionale) Crea un frame di coordinate (Assi X, Y, Z) per capire l'orientamento della camera
    # X = Rosso, Y = Verde, Z = Blu
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02, origin=[0, 0, 0])

    # Mostra a schermo: Il cubo grigio, la faccia verde, il centro rosso e gli assi
    o3d.visualization.draw_geometries([cube_cloud, face_cloud, center_sphere, axis_frame])

if __name__ == '__main__':
    main()