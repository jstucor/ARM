import sys
import numpy as np

import rclpy
from rclpy.node import Node 
from custom_interfaces.srv import Position

ARM_LENGTH = 5
ARM_CNT = 2

class SendingClientAsync(Node):
    def __init__(self):
        super().__init__("sending_client_async")
        self.client = self.create_client(Position, "position")

        while not self.client.wait_for_service(timeout_sec= 1.0):
            self.get_logger().info('waiting...')

    def send_request(self):
        request = Position.Request()

        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        servo_positions = self.inverse_kinematics(x,y,z)

        request.serv1 = float(servo_positions[0])
        request.serv2 = float(servo_positions[1])
        request.serv3 = float(servo_positions[2])
        request.serv4 = float(servo_positions[3])
        request.serv5 = float(servo_positions[4])
        request.serv6 = float(servo_positions[5])
        self.future = self.client.call_async(request)

    def inverse_kinematics(self, x, y ,z):
        offset = .5 
        servo_positions = [0,0,0,0,0,0]
        if np.sqrt(x**2 + y**2 + (z-offset)**2) > ARM_LENGTH * ARM_CNT:
            return servo_positions

        base_angle = np.degrees(np.arctan2(x,y))
        servo_positions[0] = base_angle

        a = np.sqrt(x**2 + y**2)

        ARM_sq = ARM_LENGTH**2

        h_sq =(z-offset)**2 + x**2 + y**2
        h = np.sqrt(h_sq)

        Theta = np.degrees(np.arccos(np.clip(a / h, -1.0, 1.0)))

        gamma1 = np.degrees(np.arccos(( ARM_sq + ARM_sq - h_sq)/(2 * ARM_LENGTH * ARM_LENGTH)))
        gamma2 = np.degrees(np.arccos(( ARM_sq + h_sq - ARM_sq)/(2 * ARM_LENGTH * h)))

        servo_positions[1] = gamma2 + Theta
        servo_positions[2] = gamma1

        return servo_positions





def main(args=None):
    rclpy.init(args=args)
    position_client = SendingClientAsync()
    position_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(position_client)
        if position_client.future.done():
            try:
                response = position_client.future.result()
            except Exception as e:
                position_client.get_logger().info(f"Service failed {e}")

            else:
                position_client.get_logger().info(f"Service success {response}")



            break


if __name__ == '__main__':
    main()
