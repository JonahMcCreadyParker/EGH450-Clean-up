from ultralytics import YOLO


def main():
    # Start from a small pretrained model
    model = YOLO("yolo11n.pt") 

    # Train on your Roboflow-exported dataset
    results = model.train(
        data="/Users/tajfoley/Library/CloudStorage/OneDrive-Personal/Documents/Uni/QUT/4th Year QUT/EGB349/Roboflow Exports/EGB349.v3i.yolov11/data.yaml",   # path to your Roboflow data.yaml
        epochs=20,
        imgsz=640,
        batch=8,
        name="egb349_model",
        device="mps" # remove this for a windows machine
    )
    print("Training complete.")
    print(results)

if __name__ == "__main__":
    main()