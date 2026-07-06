# This script analyzes images in a folder, extracts central patches, calculates color medians in Lab space, clusters the colors, and visualizes the results.
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import json


IMAGE_DIR = Path(__file__).parent / "cube_faces"
CROP_SIZE = 200
GRID_SIZE = 3
SAMPLE_SIZE = 5
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def center_crop_bounds(image, size):
    height, width = image.shape[:2]

    if height < size or width < size:
        raise ValueError(f"image too small: {width}x{height}, required at least {size}x{size}")

    x1 = width // 2 - size // 2
    y1 = height // 2 - size // 2
    return x1, y1, x1 + size, y1 + size


def center_crop(image, size):
    x1, y1, x2, y2 = center_crop_bounds(image, size)
    return image[y1:y2, x1:x2]


def centered_square_bounds(x1, y1, x2, y2, size):
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    half = size // 2
    return center_x - half, center_y - half, center_x - half + size, center_y - half + size


def image_with_crop_grid(image, crop_size, grid_size, sample_size):
    overlay = image.copy()
    x1, y1, x2, y2 = center_crop_bounds(image, crop_size)

    cv2.rectangle(overlay, (x1, y1), (x2 - 1, y2 - 1), (0, 0, 255), 2)

    x_edges = np.linspace(x1, x2, grid_size + 1, dtype=int)
    y_edges = np.linspace(y1, y2, grid_size + 1, dtype=int)

    for x in x_edges[1:-1]:
        cv2.line(overlay, (x, y1), (x, y2 - 1), (0, 255, 255), 2)

    for y in y_edges[1:-1]:
        cv2.line(overlay, (x1, y), (x2 - 1, y), (0, 255, 255), 2)

    for row in range(grid_size):
        for col in range(grid_size):
            sx1, sy1, sx2, sy2 = centered_square_bounds(
                x_edges[col],
                y_edges[row],
                x_edges[col + 1],
                y_edges[row + 1],
                sample_size,
            )
            cv2.rectangle(overlay, (sx1, sy1), (sx2 - 1, sy2 - 1), (255, 0, 0), 1)

    return cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)


def patch_lab_medians(crop, grid_size, sample_size):
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB).astype(np.float32)

    # OpenCV saves 8-bit Lab as L in [0, 255] and a/b with a 128 offset.
    # Here we use CIELAB coordinates: L* in [0, 100], a* and b* approximately in [-128, 127].
    lab[:, :, 0] *= 100.0 / 255.0
    lab[:, :, 1:] -= 128.0

    height, width = lab.shape[:2]
    y_edges = np.linspace(0, height, grid_size + 1, dtype=int)
    x_edges = np.linspace(0, width, grid_size + 1, dtype=int)

    medians = []
    for row in range(grid_size):
        for col in range(grid_size):
            sx1, sy1, sx2, sy2 = centered_square_bounds(
                x_edges[col],
                y_edges[row],
                x_edges[col + 1],
                y_edges[row + 1],
                sample_size,
            )
            sample = lab[sy1:sy2, sx1:sx2]
            medians.append(np.median(sample.reshape(-1, 3), axis=0))

    return np.array(medians)


def lab_points_to_rgb(points):
    lab = points.reshape(-1, 1, 3).astype(np.float32)
    rgb = cv2.cvtColor(lab, cv2.COLOR_LAB2RGB).reshape(-1, 3)
    return np.clip(rgb, 0.0, 1.0)


def cluster_lab_points(points, k=6):
    """Clusters the Lab points into k clusters using cv2.kmeans and returns the Lab centroids."""
    samples = points.astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    flags = cv2.KMEANS_PP_CENTERS
    compactness, labels, centers = cv2.kmeans(samples, k, None, criteria, 10, flags)
    return centers


def classify_patches(medians, centroids):
    """Assigns each patch to its closest centroid (nearest neighbor)."""
    # medians: (n_patches, 3), centroids: (k,3)
    dists = np.linalg.norm(medians[:, None, :] - centroids[None, :, :], axis=2)
    labels = np.argmin(dists, axis=1)
    return labels


def image_with_crop_grid_labeled(image, crop_size, grid_size, sample_size, labels=None, class_colors=None):
    """Draws the grid and, optionally, the class labels for each patch.
    `class_colors` must be a list of BGR colors (0-255) for each class.
    """
    overlay = image.copy()
    x1, y1, x2, y2 = center_crop_bounds(image, crop_size)

    cv2.rectangle(overlay, (x1, y1), (x2 - 1, y2 - 1), (0, 0, 255), 2)

    x_edges = np.linspace(x1, x2, grid_size + 1, dtype=int)
    y_edges = np.linspace(y1, y2, grid_size + 1, dtype=int)

    for x in x_edges[1:-1]:
        cv2.line(overlay, (x, y1), (x, y2 - 1), (0, 255, 255), 2)

    for y in y_edges[1:-1]:
        cv2.line(overlay, (x1, y), (x2 - 1, y), (0, 255, 255), 2)

    idx = 0
    for row in range(grid_size):
        for col in range(grid_size):
            sx1, sy1, sx2, sy2 = centered_square_bounds(
                x_edges[col],
                y_edges[row],
                x_edges[col + 1],
                y_edges[row + 1],
                sample_size,
            )
            cv2.rectangle(overlay, (sx1, sy1), (sx2 - 1, sy2 - 1), (255, 0, 0), 1)

            if labels is not None:
                cls = int(labels[idx])
                color = (0, 255, 0)
                if class_colors is not None and 0 <= cls < len(class_colors):
                    color = tuple(int(c) for c in class_colors[cls])

                # Draws a small colored rectangle next to the sample and the class number
                rect_w = max(12, sample_size)
                rx1 = sx1
                ry1 = sy1 - rect_w - 2
                if ry1 < 0:
                    ry1 = sy2 + 2
                rx2 = rx1 + rect_w
                ry2 = ry1 + rect_w
                cv2.rectangle(overlay, (rx1, ry1), (rx2, ry2), color, -1)
                text = str(cls)
                text_color = (255, 255, 255) if (color[0]*0.299 + color[1]*0.587 + color[2]*0.114) < 186 else (0, 0, 0)
                cv2.putText(overlay, text, (rx1 + 2, ry2 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv2.LINE_AA)

            idx += 1

    return cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)


def image_files(directory):
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def plot_crop_grids(previews):
    cols = min(3, len(previews))
    rows = int(np.ceil(len(previews) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3.2 * rows), squeeze=False)

    for ax, (name, preview) in zip(axes.ravel(), previews):
        ax.imshow(preview)
        ax.set_title(name, fontsize=9)
        ax.axis("off")

    for ax in axes.ravel()[len(previews):]:
        ax.axis("off")

    fig.suptitle("Central crop 200x200, grid 3x3 and samples 5x5", fontsize=13)
    plt.tight_layout()
    return fig


def main():
    if not IMAGE_DIR.exists():
        raise FileNotFoundError(f"Directory not found: {IMAGE_DIR}")

    medians_list = []
    images = []

    for image_path in image_files(IMAGE_DIR):
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"Skipping unreadable file: {image_path}")
            continue

        try:
            crop = center_crop(image, CROP_SIZE)
        except ValueError as exc:
            print(f"Skipping {image_path.name}: {exc}")
            continue

        medians = patch_lab_medians(crop, GRID_SIZE, SAMPLE_SIZE)
        medians_list.append(medians)
        images.append((image_path.name, image))

    if not medians_list:
        print("No Lab points to plot.")
        return

    points = np.vstack(medians_list)

    # Output directory
    OUTPUT_DIR = Path(__file__).parent / "outputs"
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Try to load existing centroids from JSON (preferred)
    centroids_json = OUTPUT_DIR / "centroids_lab_dict.json"
    centers = None
    if centroids_json.exists():
        try:
            with open(centroids_json, "r", encoding="utf-8") as fh:
                d = json.load(fh)
            # Sort by numerical key and create array
            keys = sorted(d.keys(), key=lambda x: int(x))
            centers = np.array([d[k] for k in keys], dtype=float)
        except Exception:
            centers = None

    # Fallback: try to load CSV if JSON is not available
    if centers is None:
        centroids_csv = OUTPUT_DIR / "centroids_lab.csv"
        if centroids_csv.exists():
            try:
                centers = np.loadtxt(centroids_csv, delimiter=",", skiprows=1)
            except Exception:
                centers = None

    # If there are no centroids, perform clustering on points and save
    if centers is None:
        K = 6
        centers = cluster_lab_points(points, K)
        centers_rgb = lab_points_to_rgb(centers)
        np.savetxt(OUTPUT_DIR / "centroids_lab.csv", centers, delimiter=",", header="L,a,b", comments="")
        np.savetxt(OUTPUT_DIR / "centroids_rgb.csv", centers_rgb, delimiter=",", header="r,g,b", comments="")

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        c=lab_points_to_rgb(points),
        s=55,
        edgecolors="black",
        linewidths=0.4,
        alpha=0.9,
    )

    ax.set_title("Lab Medians of 5x5 Samples in 3x3 Patches")
    ax.set_xlabel("L*")
    ax.set_ylabel("a*")
    ax.set_zlabel("b*")
    ax.set_xlim(0, 100)
    ax.set_ylim(-128, 127)
    ax.set_zlim(-128, 127)
    
    # Draw centroids on the scatter plot
    centers_rgb = lab_points_to_rgb(centers)
    ax.scatter(
        centers[:, 0],
        centers[:, 1],
        centers[:, 2],
        c=centers_rgb,
        s=220,
        edgecolors="black",
        linewidths=1.2,
        marker="o",
    )

    # Prepare colors for drawing (BGR 0-255)
    class_colors = (np.clip(centers_rgb * 255.0, 0, 255)).astype(int)[:, ::-1]

    # Generate labeled previews
    previews = []
    for (name, image), medians in zip(images, medians_list):
        labels = classify_patches(medians, centers)
        preview = image_with_crop_grid_labeled(image, CROP_SIZE, GRID_SIZE, SAMPLE_SIZE, labels=labels, class_colors=class_colors)
        previews.append((name, preview))

    previews_fig = plot_crop_grids(previews)

    previews_fig.savefig(OUTPUT_DIR / "previews.png", dpi=150)
    fig.savefig(OUTPUT_DIR / "lab_scatter_with_centroids.png", dpi=150)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
