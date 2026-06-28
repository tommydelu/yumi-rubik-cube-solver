import cv2
import numpy as np
from pathlib import Path
import json
from scipy.optimize import linear_sum_assignment  # For color balancing
from rubik_solver import utils  # For solving the cube via Kociemba

# --- CONFIGURATION ---
SCRIPT_DIR = Path(__file__).parent.resolve()
IMAGE_DIR = SCRIPT_DIR / "cube_faces"
JSON_PATH = SCRIPT_DIR / "centroids_lab_dict.json"

CROP_SIZE = 200
GRID_SIZE = 3
SAMPLE_SIZE = 5

# MODIFIED ORDER TO MATCH YOUR REAL INDICES:
# 0: Yellow (U) -> Center at pos 4
# 1: Blue (R)   -> Center at pos 13
# 2: Red (F)    -> Center at pos 22
# 3: Green (L)  -> Center at pos 31 (GREEN)
# 4: Orange (B) -> Center at pos 40 (ORANGE)
# 5: White (D)  -> Center at pos 49 (WHITE)
REAL_CENTER_COLORS = {
    0: "Yellow",     
    1: "Blue",        
    2: "Red",      
    3: "Green",      
    4: "Orange",  
    5: "White"      
}

# File association based on the newly requested face sequence
PHOTO_DICTIONARY = {
    0: "up.jpg",       # Yellow
    1: "right.jpg",    # Blue
    2: "front.jpg",    # Red
    3: "left.jpg",     # Green  -> will become the 4th face (indices 27-35)
    4: "back.jpg",     # Orange -> will become the 5th face (indices 36-44)
    5: "down.jpg"      # White  -> will become the 6th face (indices 45-53)
}

EN_INITIALS_TRANSLATION = {
    "White": "W",
    "Yellow": "Y",
    "Red": "R",
    "Blue": "B",
    "Green": "G",
    "Orange": "O"
}

# --- IMAGE PROCESSING FUNCTIONS ---
def center_crop_bounds(image, size):
    height, width = image.shape[:2]
    if height < size or width < size:
        raise ValueError(f"Image too small: {width}x{height}, required at least {size}x{size}")
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

def patch_lab_medians(crop, grid_size, sample_size):
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB).astype(np.float32)
    lab[:, :, 0] *= 100.0 / 255.0
    lab[:, :, 1:] -= 128.0

    height, width = lab.shape[:2]
    y_edges = np.linspace(0, height, grid_size + 1, dtype=int)
    x_edges = np.linspace(0, width, grid_size + 1, dtype=int)

    medians = []
    for row in range(grid_size):
        for col in range(grid_size):
            sx1, sy1, sx2, sy2 = centered_square_bounds(
                x_edges[col], y_edges[row], x_edges[col + 1], y_edges[row + 1], sample_size
            )
            sample = lab[sy1:sy2, sx1:sx2]
            medians.append(np.median(sample.reshape(-1, 3), axis=0))
    return np.array(medians)

# --- MAIN SCRIPT ---
def main():
    if not IMAGE_DIR.exists():
        raise FileNotFoundError(f"Directory not found: {IMAGE_DIR}")

    # Build the path list respecting your specific geometric order
    image_paths = []
    for idx in range(6):
        file_name = PHOTO_DICTIONARY[idx]
        file_path = IMAGE_DIR / file_name
        if not file_path.exists():
            print(f"[CRITICAL ERROR] Missing required file for sequence: {file_name}")
            print(f"Make sure that files with the correct names exist in the folder: {IMAGE_DIR.name}")
            return
        image_paths.append(file_path)

    faces_list = []   

    for path in image_paths:
        img = cv2.imread(str(path))
        crop = center_crop(img, CROP_SIZE)
        medians = patch_lab_medians(crop, GRID_SIZE, SAMPLE_SIZE)
        faces_list.append(medians)

    # 2. DIRECT EXTRACTION OF CENTROIDS
    json_dict = {}
    real_centroids = []
    
    for face_idx in range(6):
        color_name = REAL_CENTER_COLORS[face_idx]
        lab_center = faces_list[face_idx][4]  # Central sticker
        
        real_centroids.append(lab_center)
        json_dict[color_name] = lab_center.tolist()

    with open(JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(json_dict, f, indent=4)
    print(f"[OK] Centroid coordinates saved in: {JSON_PATH.name}")

    print("\n==================================================")
    print("    MATHEMATICAL CORRECTION AND OPTIMIZATION      ")
    print("==================================================")

    # 3. APPLICATION OF HUNGARIAN ALGORITHM (54 stickers x 6 centers)
    distance_matrix = []
    for face_idx, medians in enumerate(faces_list):
        for sticker in medians:
            distances_from_centers = [np.linalg.norm(sticker - center) for center in real_centroids]
            distance_matrix.append(distances_from_centers)
            
    distance_matrix = np.array(distance_matrix)

    # Enforce exact assignment of 9 stickers per color
    expanded_cost_matrix = np.repeat(distance_matrix, 9, axis=1)

    row_indices, col_indices = linear_sum_assignment(expanded_cost_matrix)
    corrected_predicted_colors = col_indices // 9

    # 4. RECONSTRUCT COLOR STRING STRUCTURE
    en_color_string = []

    for sticker_idx, color_idx in enumerate(corrected_predicted_colors):
        predicted_color = REAL_CENTER_COLORS[color_idx]
        en_initial = EN_INITIALS_TRANSLATION[predicted_color].lower()
        en_color_string.append(en_initial)

    # Diagnostic print of balanced grids
    FACE_LETTERS_FORM = {0: "Face 1", 1: "Face 2", 2: "Face 3", 3: "Face 4", 4: "Face 5", 5: "Face 6"}
    for face_idx in range(6):
        center_name = REAL_CENTER_COLORS[face_idx]
        position_letter = FACE_LETTERS_FORM[face_idx]
        print(f"\n{position_letter} (Real Center: {center_name}) [BALANCED]:")
        
        start = face_idx * 9
        face_colors = [REAL_CENTER_COLORS[corrected_predicted_colors[i]] for i in range(start, start + 9)]
        
        visual_grid = np.array(face_colors).reshape(3, 3)
        for row in visual_grid:
            print(f"  [ {row[0]:^11} | {row[1]:^11} | {row[2]:^11} ]")

    print("\n==================================================")
    
    total_color_string = "".join(en_color_string)
    
    print(" GENERATED COLOR STRING:")
    print("--------------------------------------------------")
    print(total_color_string)
    print("==================================================")
    
    print("Verify color counts (Must all be 9):")
    for col_char in ["y", "g", "r", "w", "b", "o"]:
        print(f"  Letter {col_char.upper()}: {total_color_string.count(col_char)}")
    print("--------------------------------------------------")
    
    # Verify specific requested center positions
    print("Verify Required Center Indices:")
    print(f"  Pos 4  (Face 1 Center - Expected Y): {total_color_string[4].upper()}")
    print(f"  Pos 13 (Face 2 Center - Expected B): {total_color_string[13].upper()}")
    print(f"  Pos 22 (Face 3 Center - Expected R): {total_color_string[22].upper()}")
    print(f"  Pos 31 (Face 4 Center - Expected G): {total_color_string[31].upper()}")
    print(f"  Pos 40 (Face 5 Center - Expected O): {total_color_string[40].upper()}")
    print(f"  Pos 49 (Face 6 Center - Expected W): {total_color_string[49].upper()}")
    print("--------------------------------------------------")
    
    # 5. SOLVER 
    try:
        print("[PROCESSING] Computing solution with rubik-solver...")
        # Note: If a custom face order is used, mapping parameters might be needed,
        # but centers are now correctly aligned.
        solution = utils.solve(total_color_string, 'Kociemba')
        
        print("\n==================================================")
        print("                 MOVES TO SOLVE                   ")
        print("==================================================")
        print(solution)
        print("==================================================")
        
    except Exception as e:
        print(f"\n[SOLVER ERROR] Unable to solve configuration.")
        print("Centers are now correctly placed according to your specifications.")
        print(f"Error detail: {e}")

if __name__ == "__main__":
    main()