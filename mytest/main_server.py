import rclpy
from rclpy.node import Node
from cmakepkgg.srv import MockSensorControl
import math



def stable_sigmoid(x):    
    if x >= 0:
        z = math.exp(-x)
        sig = 1 / (1 + z)
        return sig
    else:
        z = math.exp(x)
        sig = z / (1 + z)
        return sig




class CustomServiceServer(Node):

    def __init__(self):
        super().__init__('custom_service_server')
        self.srv = self.create_service(MockSensorControl, 'mock_sensor_control', self.service_callback)


    def service_callback(self, request, response):
        if request.sensor_id == '+':
            response.sum = request.ff1 + request.ff2
            self.get_logger().info('input "%s": %.2f + %.2f '% (request.sensor_id, request.ff1, request.ff2))
            response.is_success = True

        elif request.sensor_id == '-':
            response.sum = request.ff1 - request.ff2
            self.get_logger().info('input "%s": %.2f - %.2f '% (request.sensor_id, request.ff1, request.ff2))
            response.is_success = True

        elif request.sensor_id == 'x':
            response.sum = request.ff1*request.ff2
            self.get_logger().info('input "%s": %.2f * %.2f '% (request.sensor_id, request.ff1, request.ff2))
            response.is_success = True

        elif request.sensor_id == '/':
            response.sum = request.ff1/request.ff2
            self.get_logger().info('input "%s": %.2f / %.2f '% (request.sensor_id, request.ff1, request.ff2))
            response.is_success = True
           
        elif request.sensor_id == 'sig':
            response.sum = stable_sigmoid(request.ff1)
            self.get_logger().info('input "%s": %.2f  '% (request.sensor_id, request.ff1))
            response.is_success = True

            
        else:
            self.get_logger().info('ERROR: unknown command!')
            response.is_success = False
            
            
        return response


def main(args=None):
    rclpy.init(args=args)

    custom_service_server = CustomServiceServer()

    rclpy.spin(custom_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()