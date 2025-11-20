import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import socket
import json
import math
import time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt16
from sensor_msgs.msg import Imu  # 추가
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import base64
import subprocess
import os, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import select
sys.path.append(os.path.dirname(__file__))
from robot_config.robot_config_id import ROBOT_ID

robot_id = ROBOT_ID


def run_command(cmd, wait=True):
    print(f"[RUNNING] {cmd}")
    if wait:
        result = subprocess.run(cmd, shell=True, executable='/bin/bash')
        print(f"[DONE] Exit code: {result.returncode}")
        return result.returncode
    else:
        return subprocess.Popen(cmd, shell=True, preexec_fn=os.setpgrp, executable='/bin/bash')

# ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"
# ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: False}"


class CommandReceiver(Node):
    def __init__(self):
        super().__init__('command_receiver')
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(('0.0.0.0', 9002))
        self.last_action = None  # 마지막 명령 추적
        self.cmd_thread = threading.Thread(target=self.receive_command_loop, daemon=True)
        self.cmd_thread.start()

    def receive_command_loop(self):
        while rclpy.ok():
            try:
                ready = select.select([self.cmd_sock], [], [], 0.01)
                if ready[0]:
                    data, _ = self.cmd_sock.recvfrom(1024)
                    command = json.loads(data.decode())

                    if robot_id < 13:
                        if command.get('navigation') is not None:
                            data = command.get('navigation')
                            if data.get('action') == True:
                                print("navigation start")
                                run_command("ros2 service call /line_following/set_running std_srvs/srv/SetBool '{data: True}'", wait=False)
                            elif data.get('action') == False:
                                run_command("ros2 service call /line_following/set_running std_srvs/srv/SetBool '{data: False}'", wait=False)

                    if command.get('action') is not None:
                        action = command.get('action')
                        self.get_logger().info(f"Received command: {action}")
                        twist = Twist()
                        if action == 'move_forward':
                            twist.linear.x = 0.2
                        elif action == 'move_backward':
                            twist.linear.x = -0.2
                        elif action == 'turn_left':
                            twist.angular.z = 0.8
                        elif action == 'turn_right':
                            twist.angular.z = -0.8
                        elif action == 'stop':
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)

            except Exception as e:
                self.get_logger().error(f"Failed to receive command: {e}")

class PoseSender(Node):
    def __init__(self):
        super().__init__('pose_sender')
        self.imu_data = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host_ip = '127.0.0.1'
        self.port = 9000

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(UInt16, '/ros_robot_controller/battery', self.battery_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)  # IMU 구독 추가
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(1.0, self.lookup_transform)  # 1Hz
        self.pos = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def battery_callback(self, msg):
        self.battery = msg.data / 105.0

    def imu_callback(self, msg):
        # 최신 IMU 데이터 저장 (선형 가속도와 각속도만)
        self.imu_data = {
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            }
        }

    def odom_callback(self, msg):
        if self.imu_data is None:
            self.imu_data = {}
        self.imu_data['linear_speed'] = msg.twist.twist.linear.x
        self.imu_data['angular_speed'] = msg.twist.twist.angular.z

    def lookup_transform(self):
        try:
            # 변환 가능 여부 확인 (최대 5초 대기)
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)):
                self.get_logger().warn("Transform from 'map' to 'base_link' is not available yet.")
                return

            # 변환 데이터 가져오기
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            if not self.imu_data or \
               'linear_speed' not in self.imu_data or \
               'angular_speed' not in self.imu_data:
                self.get_logger().warn("Waiting for odometry speed data...")
                return

            t = transform.transform.translation
            q = transform.transform.rotation

            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

            self.pos['x'] = t.x
            self.pos['y'] = t.y
            self.pos['yaw'] = yaw

            data = {
                'pos': {
                    'x': self.pos['x'],
                    'y': self.pos['y'],
                    'yaw': self.pos['yaw']
                },
                'imu': self.imu_data,
                'battery': self.battery
            }

            print("\n== Transform: map → base_link ==")
            print(f"Pos X: {self.pos['x']:.3f}")
            print(f"Pos Y: {self.pos['y']:.3f}")
            print(f"Pos Yaw: {self.pos['yaw']:.3f}")

            if self.imu_data:
                print(f"IMU Acc: {self.imu_data['linear_acceleration']}")
                print(f"IMU Gyro: {self.imu_data['angular_velocity']}")
                print(f"IMU Speed - Linear: {self.imu_data['linear_speed']:.3f} m/s")
                print(f"IMU Speed - Angular: {self.imu_data['angular_speed']:.3f} rad/s")

            print(f"Battery Level: {self.battery:.2f}%" if self.battery is not None else "Battery Level: N/A")

            self.sock.sendto(json.dumps(data).encode(), (self.host_ip, self.port))

        except Exception as e:
            self.get_logger().error(f'Could not transform: {e}')

class CameraSender(Node):
    def __init__(self):
        super().__init__('camera_sender')
        self.bridge = CvBridge()
        self.camera_frame = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.host_ip = '127.0.0.1'
        self.host_ip = '10.243.76.38'
        self.port = 9001  # separate port for camera

        self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.camera_callback, 10)
        # 10Hz
        # self.create_timer(0.1, self.send_image)
        # 60Hz
        self.create_timer(1.0 / 30.0, self.send_image)
        # 120Hz
        # self.create_timer(1.0 / 60.0, self.send_image)

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (320, 240))  # 예: 320x240으로 축소
            # cv_image = cv2.resize(cv_image, (160, 120))  # 예: 320x240으로 축소

            # Set JPEG encoding quality to 95 to reduce compression variability
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, jpeg = cv2.imencode('.jpg', cv_image, encode_param)
            # _, jpeg = cv2.imencode('.jpg', cv_image)  # original line removed/commented
            # with encoding
            # b64_image = base64.b64encode(jpeg).decode('utf-8')
            # self.camera_frame = b64_image

            self.camera_frame = jpeg.tobytes()  # JPEG 이미지 바이트로 저장A
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def send_image(self):
        if self.camera_frame:
            robot_id_bytes = int(robot_id).to_bytes(2, byteorder='big')  # 0~65535 가능
            jpeg_bytes = self.camera_frame

            # 고정 크기로 만들기 (optional)
            TARGET_SIZE = 30720
            payload = robot_id_bytes + jpeg_bytes
            if len(payload) < TARGET_SIZE:
                payload += b'\x00' * (TARGET_SIZE - len(payload))

            try:
                self.sock.sendto(payload, (self.host_ip, self.port))
            except Exception as e:
                self.get_logger().error(f"Failed to send image frame: {e}")

    # def send_image(self):
    #     TARGET_SIZE = 30720  # 전체 전송 데이터 목표 크기(30KB)
    #     if self.camera_frame:
    #         b64_image = self.camera_frame
    #         dummy_len = max(0, TARGET_SIZE - len(b64_image))
    #         data = {
    #             'robot_id': str(robot_id),
    #             'image': b64_image,
    #             # 'dummy': '0' * dummy_len  # 부족한 만큼 dummy 데이터 추가
    #         }

    #         try:
    #             self.sock.sendto(json.dumps(data).encode(), (self.host_ip, self.port))
    #             # self.get_logger().info(f"Sent camera frame (image: {len(b64_image)}, dummy: {dummy_len})")
    #         except Exception as e:
    #             self.get_logger().error(f"Failed to send camera frame: {e}")

# ros2 service call /line_following/enter std_srvs/srv/Trigger {}
# ros2 service call /line_following/force_pick_color std_srvs/srv/Trigger "{}"
# ros2 service call /line_following/set_threshold interfaces/srv/SetFloat64 "{data: 0.5}"

# ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"
# ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: False}"

# def main():
#     rclpy.init()
#     pose_node = PoseSender()
#     camera_node = CameraSender()
#     cmd_node = CommandReceiver()
#     from line_following import LineFollowingNode
#     line_node = LineFollowingNode('line_following')


#     # run_command("ros2 service call /line_following/set_threshold interfaces/srv/SetFloat64 \"{data: 0.5}\"", wait=False)
#     run_command("ros2 service call /line_following/enter std_srvs/srv/Trigger '{}'", wait=False)
#     time.sleep(3)
#     run_command("ros2 service call /line_following/force_pick_color std_srvs/srv/Trigger '{}'", wait=False)
#     time.sleep(3)

#     try:
#         executor = rclpy.executors.MultiThreadedExecutor()
#         executor.add_node(pose_node)
#         executor.add_node(camera_node)
#         executor.add_node(cmd_node)
#         executor.add_node(line_node)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass

    # pose_node.destroy_node()
    # camera_node.destroy_node()
    # cmd_node.destroy_node()
    # line_node.destroy_node()
    # rclpy.shutdown()


def main():
    rclpy.init()

    # 여러 노드 생성
    pose_node = PoseSender()
    camera_node = CameraSender()
    cmd_node = CommandReceiver()
    from line_following import LineFollowingNode
    line_node = LineFollowingNode('line_following')

    # MultiThreadedExecutor 사용
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pose_node)
    executor.add_node(camera_node)
    executor.add_node(cmd_node)
    if robot_id < 13:
        executor.add_node(line_node)

    # spin 전에 서비스 호출 넣으면 의미 없음
    # 대신 스레드로 spin 이후에 호출되도록 한다
    import threading, time
    def call_services_later():
        time.sleep(3)
        run_command("ros2 service call /line_following/enter std_srvs/srv/Trigger '{}'", wait=True)
        time.sleep(3)
        run_command("ros2 service call /line_following/force_pick_color std_srvs/srv/Trigger '{}'", wait=False)

    threading.Thread(target=call_services_later, daemon=True).start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in [pose_node, camera_node, cmd_node, line_node]:
            node.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()