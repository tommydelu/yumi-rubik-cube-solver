# This script allows you to take photos with your webcam and save them to a specific folder.
# The red square in the center of the screen indicates the area where you should position your face for the photo.
# Press 's' to save the photo, and press 'q' to exit the program.
import cv2
import os

SAVE_DIR = "sequence_1"
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open the webcam.")
    exit()

existing_files = [
    f for f in os.listdir(SAVE_DIR)
    if f.startswith("face_") and f.endswith(".jpg")
]

counter = len(existing_files) + 1

print("Press 's' to save a photo")
print("Press 'q' to exit")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error reading from the webcam.")
        break

    height, width = frame.shape[:2]

    # Square size
    square_size = 200

    # Coordinates of the centered square
    x1 = width // 2 - square_size // 2
    y1 = height // 2 - square_size // 2
    x2 = width // 2 + square_size // 2
    y2 = height // 2 + square_size // 2

    # Draw red square
    cv2.rectangle(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 0, 255),
        2
    )

    cv2.imshow("Webcam", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        filename = f"face_{counter:02d}.jpg"
        filepath = os.path.join(SAVE_DIR, filename)

        cv2.imwrite(filepath, frame)
        print(f"Photo saved: {filepath}")

        counter += 1

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()