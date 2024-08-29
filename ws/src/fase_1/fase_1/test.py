import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library

model = YOLO("colab_model.pt")
# Convert ROS Image message to OpenCV image
current_frame = cv2.imread("img2.jpg")  # Load the image from the file

# Resize the frame
resized_frame = cv2.resize(current_frame, (880, 680))  # Adjust the size as needed

# Run YOLO model on the frame
results = model(resized_frame, conf=0.9)

print(results[0].boxes)

# Visualize the results on the frame
annotated_frame = results[0].plot()  # YOLOv8 automatically annotates the frame

posicoes = [
    (
        int((caixa.xyxy[0][0] + caixa.xyxy[0][2]) / 2),
        int((caixa.xyxy[0][1] + caixa.xyxy[0][3]) / 2),
    )
    for caixa in results[0].boxes
]

# annotate point in each item os posicoes
for posicao in posicoes:
    cv2.circle(annotated_frame, posicao, 5, (0, 0, 255), -1)

# Show Results
cv2.imshow("Camera Feed", annotated_frame)
while True:
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
