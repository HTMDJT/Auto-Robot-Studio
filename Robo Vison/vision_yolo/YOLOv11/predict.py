from ultralytics import YOLO

model = YOLO("yolo11n.pt")

model.predict(
    source = "ultralytics/assets",
    show = True,
    save = True
)