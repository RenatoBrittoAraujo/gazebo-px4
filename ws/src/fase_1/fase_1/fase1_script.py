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
    VehicleLandDetected,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from time import sleep
from ultralytics import YOLO  # YOLO library
import os

SHOULD_SHOW_COMMON_LOGS = False
global li
li = 0


def should_print_frequent_log():
    global li
    if SHOULD_SHOW_COMMON_LOGS:
        return True
    if li % 80 == 0:
        return True
    return False


def distance(coord1, coord2):
    return ((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2) ** 0.5


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("trainee")

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

        self.camera_subscription = self.create_subscription(
            Image, "camera", self.camera_boxes_callback, 10
        )
        self.camera_subscription  # prevent unused variable warning

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.bridge = CvBridge()
        self.model = YOLO("src/fase_1/fase_1/colab_model.pt", verbose=False)

        self.takeoff_height = -3.5
        self.setpoints = []
        self.current_setpoint_index = 18

        # CRIA 64 SETPOINTS PELA ARENA 8x8
        for column in range(0, 8):
            for row in range(0, 8):
                # offset para centralizar drone no quadrado
                self.setpoints.append((-column + 0.1, row + 0.3, self.takeoff_height))

        self.boxes = []
        self.capture_picture_timer = 0

        self.pending_landing_final_approach = False
        self.landed_timeout = 0

        self.capture_image_timer = 1000000000  # 1 seconds
        self.land_wait = 15000000000  # 15 seconds
        self.last_landing_spot = (0, 0, 0)
        self.override_pos = (0, 0, 0)

        self.save_img_trigger = False
        self.detection_to_be_processed = False
        self.first_call = True
        self.awaiting_takeoff = False

        # delete files in src/fase_1/fase_1/saved/
        for file in os.listdir("src/fase_1/fase_1/saved/"):
            os.remove(f"src/fase_1/fase_1/saved/{file}")

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        if should_print_frequent_log():
            self.get_logger().info(
                f"Current position: x={vehicle_local_position.x}, y={vehicle_local_position.y}, z={vehicle_local_position.z}"
            )

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

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

    def takeoff(self):
        """Send a takeoff command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_VTOL_TAKEOFF)
        self.get_logger().info("Takeoff command sent")

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
        if should_print_frequent_log():
            self.get_logger().info(f"Publishing position setpoint: {[x, y, z]}")
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
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

    def camera_boxes_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        resized_frame = cv2.resize(current_frame, (880, 680))
        results = self.model(resized_frame, conf=0.9, verbose=False)
        annotated_frame = results[0].plot()

        # calcula centro de todas as caixas de deteção no frame
        boxes = [
            (
                int((caixa.xyxy[0][0] + caixa.xyxy[0][2]) / 2),
                int((caixa.xyxy[0][1] + caixa.xyxy[0][3]) / 2),
            )
            for caixa in results[0].boxes
        ]
        self.boxes = boxes  # CAIXAS DETECTADAS

        # draw center of boxes in boxes
        for box in boxes:
            cv2.circle(annotated_frame, box, 5, (0, 0, 255), -1)

        # draw center of image
        cv2.circle(annotated_frame, (880 // 2, 680 // 2), 5, (0, 255, 0), -1)

        if self.save_img_trigger:
            self.save_img_trigger = False

            sp = self.setpoints[self.current_setpoint_index]

            cv2.imwrite(
                f"src/fase_1/fase_1/saved/{round(sp[0])}_{round(sp[1])}_{round(sp[2])}.png",
                annotated_frame,
            )

        # Show Results
        cv2.imshow("Camera Feed", annotated_frame)
        cv2.waitKey(1)

    def set_landing_timeout(self):
        self.landed_timeout = self.get_clock().now().nanoseconds + self.land_wait

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        global li
        li += 1

        self.publish_offboard_control_heartbeat_signal()

        # rodar na primeira chamada
        if self.first_call:
            self.first_call = False

            # seta o spawn como landing spot
            self.last_landing_spot = (
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z,
            )

            self.engage_offboard_mode()
            self.arm()
            # self.takeoff()

            return

        # se o drone estiver aproximando da base para pouso ou pousando
        if (
            self.landed_timeout - self.get_clock().now().nanoseconds >= 0
            or self.pending_landing_final_approach
        ):
            self.get_logger().info("LANDED OR LANDING AND WAITING...")
            self.controla_pouso()
            return

        # se o drone estiver pousado e está esperando para decolar
        if (
            self.landed_timeout - self.get_clock().now().nanoseconds < 0
            and self.awaiting_takeoff
        ):
            self.awaiting_takeoff = False
            self.takeoff()
            return

        sp = self.setpoints[self.current_setpoint_index]
        self.publish_position_setpoint(sp[0], sp[1], sp[2])

        # se o drone está no setpoint
        if (
            abs(self.vehicle_local_position.x - sp[0]) < 0.12
            and abs(self.vehicle_local_position.y - sp[1]) < 0.12
            and abs(self.vehicle_local_position.z - sp[2]) < 0.12
        ):
            self.get_logger().info(f"Moving to setpoint: {sp}")
            current_time_delta = (
                self.get_clock().now().nanoseconds - self.capture_picture_timer
            )

            # drone está no setpoint sem timer inciado
            if (
                current_time_delta >= self.capture_image_timer
                and not self.detection_to_be_processed
            ):
                self.get_logger().info(
                    f"Criando temporizador para processar detecção de imagem"
                )

                self.detection_to_be_processed = True
                self.capture_picture_timer = self.get_clock().now().nanoseconds
                self.save_img_trigger = True
                return

            # drone está no setpoint esperando timer
            if current_time_delta < self.capture_image_timer:
                self.get_logger().info(f"Esperando tirar imagem")
                return

            # drone está no setpoint depois do fim do timer
            if (
                current_time_delta >= self.capture_image_timer
                and self.detection_to_be_processed
            ):
                self.get_logger().info(
                    f"Temporizador de processamento de imagem finalizado"
                )

                self.detection_to_be_processed = False
                self.current_setpoint_index = (self.current_setpoint_index + 1) % len(
                    self.setpoints
                )
                self.get_logger().info(f"Moving to new setpoint: {sp}")

                # Passa controle para o sistema de pouso case exista base na imagem tirada
                self.controla_pouso()
                return

    def controla_pouso(self):
        coord_drone = (
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z,
        )
        caixas = self.boxes

        sp = self.setpoints[self.current_setpoint_index]

        self.get_logger().info(
            f"controla_pouso CHAMADO EM {round(sp[0])}_{round(sp[1])}_{round(sp[2])}\nVARIÁVEIS: coord_drone"
            + repr(coord_drone)
            + " caixas:"
            + repr(caixas)
        )

        # =============== SETANDO VARIAVEIS
        center_img = (880 / 2, 680 / 2)
        off_center_min = 0.1 * center_img[0] * 2
        off_center_max = 0.4 * center_img[0] * 2
        landing_tolerance = 0.03 * center_img[0] * 2
        landing_zeroin_delta = 0.5
        last_landing_dist = 0

        has_caixa = len(caixas) > 0
        closest = None
        dist_closest = None
        if has_caixa:
            closest = sorted(caixas, key=lambda caixa: distance(caixa, coord_drone))[0]
            dist_closest = distance(
                coord_drone,
                sorted(caixas, key=lambda caixa: distance(caixa, coord_drone))[0],
            )

        # ================ EM ESTADO DE POUSAR
        if self.pending_landing_final_approach:

            # --------------> Avaliando oportunidade para pousar
            if not has_caixa:
                self.get_logger().info("ESPERANDO PORQUE NAO HA CAIXAS DETECTADAS")
            elif distance(closest, center_img) < landing_tolerance:
                self.get_logger().info("POUSO AUTORIZADO")
                self.land()
                self.pending_landing_final_approach = False
                self.awaiting_takeoff = True
                self.set_landing_timeout()

            # --------------> INDO NA DIRECAO DA BASE MAIS PROXIMA
            else:
                self.get_logger().info("INDO NA DIRECAO DA BASE MAIS PROXIMA")
                # vector of coord_drone to closest caixa

                self.get_logger().info(
                    "closest, coord_drone, center_img: "
                    + repr(closest)
                    + " "
                    + repr(coord_drone)
                    + " "
                    + repr(center_img)
                )

                vec = (
                    (closest[0] - center_img[0]) / dist_closest,
                    (closest[1] - center_img[1]) / dist_closest,
                )

                self.override_pos = (
                    coord_drone[0] - vec[0] * landing_zeroin_delta,
                    coord_drone[1] - vec[1] * landing_zeroin_delta,
                )
                self.get_logger().info(
                    "VEC, OVERRIDE_POS: " + repr(vec) + " " + repr(self.override_pos)
                )

                # aproxima um pouco o drone do chao
                altura = self.takeoff_height + 1

                # move na direção de proximidade com centro da caixa
                self.publish_position_setpoint(
                    self.override_pos[0], self.override_pos[1], altura
                )

            return

        # ================ EM ESTADO DE PROCURAR BASE DE POUSO
        elif has_caixa:

            # se a caixa mais proxima estiver muito proxima da ultima base de pouso (assumimos que é a mesma base)
            if distance(coord_drone, self.last_landing_spot) < last_landing_dist:
                self.get_logger().info("TOO CLOSE TO LAST LANDING, SKIP...")
                return

            dist = distance(closest, center_img)

            # se a caixa mais proxima estiver na distancia certa do centro da imagem
            if dist > off_center_min and dist < off_center_max:
                self.get_logger().info("BASE DE POUSO ENCONTRADA, INICIANDO POUSO...")
                self.pending_landing_final_approach = True
                return

            # se a distancia do centro da imagem estiver grande, ignora
            else:
                self.get_logger().info("DISTANCIA INVALIDA: " + str(dist))
                self.get_logger().info(
                    repr(closest)
                    + " "
                    + repr(center_img)
                    + " "
                    + repr(off_center_min)
                    + " "
                    + repr(off_center_max)
                )

        else:
            self.get_logger().info("NAO HA CAIXAS DETECTADAS!")


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
