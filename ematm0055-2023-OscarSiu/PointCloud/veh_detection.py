from ultralytics import YOLO
import cv2
import torch

# Load pre-trained YOLOv8 model
model = YOLO("yolov8n.pt")  # You can use 'yolov8s.pt' for a more accurate model

# Load image
image_path = r"sensor_data\straightline_rain_10\images\00004.png"
img = cv2.imread(image_path)

if img is None:
    raise ValueError("Error: Could not load image. Check file path and format.")

# Run inference
results = model(image_path)

for result in results:
    for box in result.boxes:
        class_id = int(box.cls[0])  # Get class ID
        if class_id in [2, 3, 5, 7]:  # COCO classes: 2(car), 3(motorcycle), 5(bus), 7(truck)
            print(f"Detected vehicle at: {box.xyxy}")  # Bounding box coordinates
            
            bbox = box.xyxy[0]  # Extract bounding box tensor
            x_min, y_min, x_max, y_max = map(int, bbox)  # Convert to integers
            
             # Calculate width and height
            width = x_max - x_min
            height = y_max - y_min
            
            print(f"Width: {width} pixels, Height: {height} pixels\n")
            
            # Draw bounding box on the image
            cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(img, "Vehicle", (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
# Show image with bounding boxes
cv2.imshow("Detected Vehicles", img)
cv2.waitKey(0)
cv2.destroyAllWindows()