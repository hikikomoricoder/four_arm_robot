from ultralytics import YOLO

# Load a pretrained YOLO11n model
model = YOLO("./runs/detect/distill11/weights/gazebo_room_coco.pt")

# Export the model to ONNX format for deployment
path = model.export(format="onnx",imgsz=480,opset=21)  # Returns the path to the exported model