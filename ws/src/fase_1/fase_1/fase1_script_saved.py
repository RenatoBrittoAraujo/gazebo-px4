#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from time import sleep

# from sensor_msgs.msg import Image  # Image is the message type
# from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
# from ultralytics import YOLO  # YOLO library
# import cv2  # OpenCV library
# import os


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("offboard_control_takeoff_and_setpoints")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        # Subscriber for camera image
        self.camera_subscription = self.create_subscription(
            Image, "/x500_mono_cam/camera/image_raw", self.camera_callback, 10
        )

        self.bridge = CvBridge()

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -3.5
        self.setpoints = []
        # for i in range(0, 8):
        #     if i % 2 == 1:
        #         for j in range(7, -1, -1):
        #             self.setpoints.append((-i + 0.5, j + 0.5, self.takeoff_height))
        #     else:
        #         for j in range(0, 8):
        #             self.setpoints.append((-i + 0.5, j + 0.5, self.takeoff_height))

        for i in range(0, 8):
            if i % 2 == 1:
                for j in range(7, -1, -1):
                    self.setpoints.append((-i, j, self.takeoff_height))
                self.get_logger().info(f"ADD SETPOINT: " + repr(self.setpoints[-1]))
            else:
                for j in range(0, 8):
                    self.setpoints.append((-i, j, self.takeoff_height))
                self.get_logger().info(f"ADD SETPOINT: " + repr(self.setpoints[-1]))

        # self.setpoints = [
        #     (0.0, 0.0, self.takeoff_height),  # Start at the origin
        #     (-4.0, 0.0, self.takeoff_height),  # Move to (4, 0)
        #     (-4.0, 4.0, self.takeoff_height),  # Move to (4, 4)
        #     (0.0, -4.0, self.takeoff_height),  # Move to (0, 4)
        #     (0.0, 0.0, self.takeoff_height),  # Return to the origin
        # ]
        self.current_setpoint_index = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # # Create the subscriber. This subscriber will receive an Image
        # # from the camera topic. The queue size is 10 messages.
        # self.subscription = self.create_subscription(
        #     Image, "camera", self.listener_callback, 10
        # )
        # self.subscription  # prevent unused variable warning

        # # Used to convert between ROS and OpenCV images
        # self.br = CvBridge()

        # self.get_logger().info(f"PATH:" + os.getcwd())

        # # atual: /edra/adcode/cbr_ws
        # # necessário /edra/adcode/cbr_ws/install/fase_1/lib/fase_1/fase1_script
        # # necessario:

        # # Load the YOLO model
        # self.model = YOLO("/edra/adcode/cbr_ws/install/fase_1/lib/fase_1/fase1_script")

        self.posicoes = []

        self.last_timestamp = 0
        self.detection_to_be_processed = False

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        self.get_logger().info(
            f"Current position: x={vehicle_local_position.x}, y={vehicle_local_position.y}, z={vehicle_local_position.z}"
        )

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def camera_callback(self, msg):
        """Callback function for camera image topic subscriber."""
        self.get_logger().info("Receiving video frame")
        return

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Resize the frame
        resized_frame = cv2.resize(
            current_frame, (880, 680)
        )  # Adjust the size as needed

        # Run YOLO model on the frame
        results = self.model(resized_frame, conf=0.9)

        # # Visualize the results on the frame
        # annotated_frame = results[0].plot()  # YOLOv8 automatically annotates the frame

        caixas = results[0].boxes[0].xyxy
        self.get_logger().info(
            str(len(results[0]))
            + " boxes: ".join(
                [repr(results[0].boxes[i].xyxy) for i in range(len(results[0].boxes))]
            )
        )

        posicoes = [
            (
                int((caixa.xyxy[0][0] + caixa.xyxy[0][2]) / 2),
                int((caixa.xyxy[0][1] + caixa.xyxy[0][3]) / 2),
            )
            for caixa in results[0].boxes
        ]
        self.get_logger().info(
            "posicoes: ".join([repr(posicao) for posicao in posicoes])
        )
        self.get_logger().info(str(posicoes))

        self.get_logger().info("SET POSICOES ==================")
        self.posicoes = posicoes

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0
        )
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        self.get_logger().info(f"Publishing position setpoint: {[x, y, z]}")
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"self.trajectory_setpoint_publisher.publish")
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter > 10:
            if self.current_setpoint_index < len(self.setpoints):
                sp = self.setpoints[self.current_setpoint_index]
                self.publish_position_setpoint(sp[0], sp[1], sp[2])
                self.get_logger().info("PUBLICOU SETPOINT")
                if (
                    abs(self.vehicle_local_position.x - sp[0]) < 0.5
                    and abs(self.vehicle_local_position.y - sp[1]) < 0.5
                    and abs(self.vehicle_local_position.z - sp[2]) < 0.5
                ):
                    self.current_setpoint_index += 1
                    self.get_logger().info(f"Moved to setpoint: {sp}")
                    # self.get_logger().info("CHEGOU NO SETPOINT")
                    # current_time_delta = (
                    #     self.get_clock().now().nanoseconds - self.last_timestamp
                    # )
                    # timer = 5000000000

                    # if current_time_delta < timer:
                    #     self.get_logger().info(
                    #         f"WAITING TIMER HIT 5{current_time_delta}"
                    #     )
                    #     return

                    # if (
                    #     current_time_delta >= timer
                    #     and not self.detection_to_be_processed
                    # ):
                    #     self.get_logger().info(f"SET TIMER 5")
                    #     self.detection_to_be_processed = True
                    #     self.last_timestamp = self.get_clock().now().nanoseconds

                    # if current_time_delta >= timer and self.detection_to_be_processed:
                    #     self.get_logger().info(f"TIMER HAS HIT 5")
                    #     self.detection_to_be_processed = False
                    #     self.detect_caixa(
                    #         (
                    #             self.vehicle_local_position.x,
                    #             self.vehicle_local_position.y,
                    #             self.vehicle_local_position.z,
                    #         ),
                    #         self.posicoes,
                    #     )
                    #     self.current_setpoint_index += 1
                    #     self.get_logger().info(f"Moved to setpoint: {sp}")
                sleep(0.1)
            else:
                self.current_setpoint_index = 0
                # self.land()
                # self.disarm()
                # rclpy.shutdown()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def detect_caixa(self, coord_drone, caixas):
        self.get_logger().info(
            "detect_caixa() " + repr(coord_drone) + " " + repr(caixas)
        )

        # caixas = self.tira_foto()
        # for caixa in caixas:
        #     caixa_xyxy
        #     if centro caixa está a menos de 1.5 metro do centro image:
        #         continue
        #     p = caixa_delta_x / caixa_delta_y
        #     if p > 1.3 or p < .7:
        #         continue
        #     if caixa pra direita da imagem:
        #         continue
        #     coord_drone
        #     xyxy = [dim_caixa + coord_drone for dim_caixa in caixa_xyxy]
        #     if caixa tem overlap de 25% de área qqr outra caixa:
        #         continue
        #     # aprovado
        #     caixas.append(xyxy)


# def func():
#     takeoff()

#     # manter orientação para cond de direita ou esquerda
#     # esperar estabilizar entre cada movimento

#     wp = []
#     for i in range(0,8):
#         if i % 2 == 1:
#             for j in range(7,-1,-1):
#                 wp.append((i+.5, j+.5))
#         else:
#             for j in range(0,8):
#                 wp.append((i+.5, j+.5))

#     print(wp)

# for point in wp:
#         goto(point)
#         caixas = tira_foto()
#         for caixa in caixas:
#             caixa_xyxy
#             if centro caixa está a menos de 1.5 metro do centro image:
#                 continue
#             p = caixa_delta_x / caixa_delta_y
#             if p > 1.3 or p < .7:
#                 continue
#             if caixa pra direita da imagem:
#                 continue
#             coord_drone
#             xyxy = [dim_caixa + coord_drone for dim_caixa in caixa_xyxy]
#             if caixa tem overlap de 25% de área qqr outra caixa:
#                 continue
#             # aprovado
#             caixas.append(xyxy)

#     for caixa in caixas:
#         goto(caixa)
#         land()
#         takeoff()

#     goto(.5,.5)
#     land()


def main(args=None) -> None:
    print("Starting offboard control node...")
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
