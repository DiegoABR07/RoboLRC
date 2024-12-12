# Import necessary libraries
from ultralytics import YOLO
import cv2  # OpenCV for image processing
import numpy as np

def load_model(model_path):
    """Load the YOLO model for detection."""
    return YOLO(model_path)

def run_inference(model, image, confTh):
    """Run inference on an image using the YOLO model."""
    results = model(image, conf=confTh)
    return results

def find_closest_object(result):
    """Identify the closest object based on the centroid's Y-coordinate."""
    boxes = result.boxes.xyxy.cpu().numpy()  # Bounding boxes
    if boxes.size == 0:
        return None  # No objects detected

    # Compute centroids
    centroids = (boxes[:, :2] + boxes[:, 2:]) / 2
    cy = centroids[:, 1]  # Y-coordinates of centroids

    # Find the index of the maximum Y-coordinate
    max_index = np.argmax(cy)
    closest_box = boxes[max_index]
    return closest_box

def display_results(result, image, closest_box, class_names):
    """Display detection results on the image with the closest object highlighted in red."""
    boxes = result.boxes.xyxy.cpu().numpy().astype(int)
    classes = result.boxes.cls.cpu().numpy().astype(int)

    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = box
        label = class_names[classes[i]]

        if closest_box is not None and np.array_equal(box, closest_box.astype(int)):
            # Draw a filled red rectangle for the closest object
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), thickness=-1)
            cv2.putText(image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            # Draw a regular bounding box for other objects
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), thickness=2)
            cv2.putText(image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return image

def main(model_path, confTh):
    """Main function to run the YOLO detection pipeline and highlight the closest object in real-time."""
    # Load model
    model = load_model(model_path)

    # Open video capture (0 for default webcam)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame from video capture.")
            break

        # Resize the frame for faster processing
        frame = cv2.resize(frame, (640, 480))

        # Run inference on the frame
        results = run_inference(model, frame, confTh)

        if results:
            # Since we're processing a single frame, get the first result
            result = results[0]

            # Find the closest object to the bottom of the frame
            closest_box = find_closest_object(result)

            # Display results with the closest object highlighted
            output_frame = display_results(result, frame, closest_box, model.names)
        else:
            output_frame = frame

        # Show the output frame
        cv2.imshow("YOLO Detection with Closest Object Highlighted", output_frame)

        # Exit if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release video capture and close windows
    cap.release()
    cv2.destroyAllWindows()

# Set the model path and confidence threshold
model_path = "yolov8s-oiv7-custom-ME.pt"
# model_path = "yolov11n-custom-ME_640.engine"
confTh = 0.65

# Run the main function
main(model_path, confTh)
