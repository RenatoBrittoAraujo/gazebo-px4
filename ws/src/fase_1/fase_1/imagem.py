# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_subscriber")

        # Create the subscriber. This subscriber will receive an Image
        # from the camera topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, "camera", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Load the YOLO model
        self.model = YOLO("colab_model.pt")

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Resize the frame
        resized_frame = cv2.resize(
            current_frame, (880, 680)
        )  # Adjust the size as needed

        # Run YOLO model on the frame
        results = self.model(resized_frame, conf=0.9)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()  # YOLOv8 automatically annotates the frame

        # caixas = results[0].boxes[0].xyxy
        # self.get_logger().info(
        #     str(len(results[0]))
        #     + " boxes: ".join(
        #         [repr(results[0].boxes[i].xyxy) for i in range(len(results[0].boxes))]
        #     )
        # )

        # posicoes = [
        #     (
        #         int((caixa.xyxy[0][0] + caixa.xyxy[0][2]) / 2),
        #         int((caixa.xyxy[0][1] + caixa.xyxy[0][3]) / 2),
        #     )
        #     for caixa in results[0].boxes
        # ]
        # self.get_logger().info(str(posicoes))

        # Show Results
        cv2.imshow("Camera Feed", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
