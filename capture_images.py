import cv2

# Open the video capture device (0 is usually the default camera)
camera = cv2.VideoCapture(0)

# Initialize camera feed
IMG_SIZE = (640, 480)
camera.set(3, IMG_SIZE[0])
camera.set(4, IMG_SIZE[1])

# Check if the camera is opened successfully
if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

# Initialize variables
frame_count = 0
save_path = "captured_frames/"

# Create the folder to save frames if it doesn't exist
import os
os.makedirs(save_path, exist_ok=True)

while True:
    # Read a frame from the camera
    ret, frame = camera.read()
    
    # Resize the images
    resized_image = cv2.resize(frame, IMG_SIZE)

    # Check if the frame was read successfully
    if not ret:
        print("Error: Failed to camerature frame.")
        break

    # Display the frame
    cv2.imshow('Resized Feed', resized_image)

    # Check if the space bar is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        # Save the frame
        frame_count += 1
        frame_filename = f"{save_path}frame_{frame_count}.png"
        cv2.imwrite(frame_filename, resized_image)
        print(f"Frame saved: {frame_filename}")

    # Break the loop if the 'q' key is pressed
    elif key == ord('q'):
        break

# Release the video camerature object and close the OpenCV windows
camera.release()
cv2.destroyAllWindows()