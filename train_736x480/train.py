from ultralytics import YOLO

model = YOLO('yolov8.yaml')

if __name__ == '__main__':
    model.train(data="config.yaml", imgsz=[480,736], rect=True, epochs=250)
