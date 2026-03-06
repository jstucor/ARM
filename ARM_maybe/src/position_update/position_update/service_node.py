import rclpy 
from rclpy.node import Node
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

# Defining the motor pins
SERVO_1 = 10 
SERVO_2 = 11
SERVO_3 = 12
SERVO_4 = 13


from custom_interfaces.srv import Position

class PositionService(Node):
    def __init__(self):
        super().__init__("position_service")

        self.service = self.create_service( #Actually making the service
                Position,
                "position",
                self.position_callback 
                )

            # Motor initializations
        self.factory = PiGPIOFactory()
        self.servo_one = AngularServo(SERVO_1,min_angle=-90, max_angle=90, pin_factory=self.factory)
        self.servo_two = AngularServo(SERVO_2,min_angle=-90, max_angle=90, pin_factory=self.factory)

    def position_callback(self, request, response):
        self.get_logger().info(f"Received request: {request.serv1}, {request.serv2}")
            
            # Writing motor positions to the servos
        self.servo_one.angle = float(request.serv1)
        self.servo_two.angle = float(request.serv2)


        response.success_code = 100 
        self.get_logger().info(f"Request received, moving motors")
        return response


def main(args=None):
    rclpy.init(args=args)
    #create node
    position_service = PositionService()
    

    #use node
    rclpy.spin(position_service)

    position_service.destroy_node()
    rclpy.shutdown()
    
    



if __name__ == "__main__":
    main()
