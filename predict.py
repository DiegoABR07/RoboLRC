from ultralytics import YOLO

# Segment
# yolo task=segment mode=train epochs=50 data=./Dataset/dataset.yaml model=yolov8s-qseg.pt imgsz=640 batch=4
# model = YOLO("yolov8s-seg-custom.pt")

# Detect
# yolo task=detect mode=train epochs=15 data=./Dataset_2/dataset.yaml model=yolov8s-oiv7.pt imgsz=640 batch=4
model = YOLO("yolov8s-oiv7-custom-ME.pt")
# model = YOLO("yolov11s-custom-ME.pt")

model.predict(source="3.jpg", show=True, save=True, show_labels=True ,show_conf=True, conf=0.65, save_txt=False, save_crop=False, line_width=2)
