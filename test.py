from ultralytics import YOLO

imgsz = (480, 480)

# Load the exported TFLite model
tflite_model = YOLO("yolov11n-custom-ME_480_saved_model/yolov11n-custom-ME_480_dynamic_range_quant.tflite", task='detect')

# Run inference
results = tflite_model("1.jpg", imgsz=480, show=True, save=True, show_labels=True ,show_conf=True, conf=0.65, save_txt=False, save_crop=False, line_width=2)
