import sys

from cmakepkgg.srv import MockSensorControl
import rclpy
from rclpy.node import Node


class Sigmoide(Node):

    def __init__(self):
        super().__init__('sigmoide_fun')
        self.client = self.create_client(MockSensorControl, 'mock_sensor_control') # Client builder 패턴
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for custom service server...')
        self.req = MockSensorControl.Request()

    def send_request(self):
        self.req.sensor_id = str(sys.argv[1])
        self.req.ff1 = float(sys.argv[2])
        self.future = self.client.call_async(self.req)
    


def main(args=None):
    rclpy.init(args=args)

    sigmoide = Sigmoide()
    sigmoide.send_request()

    while rclpy.ok():
        rclpy.spin_once(sigmoide)
        # future가 done 상태가 되면 다음 if문을 실행합니다
        if sigmoide.future.done():
            try:
                response = sigmoide.future.result()
            except Exception as e:

                sigmoide.get_logger().info(
                    'Service call failed: %r' % (e,))
            else:
                sigmoide.get_logger().info(
                    'input: %.2f sigmoid %.6f' %
                    (sigmoide.req.ff1,response.sum))
            break

    sigmoide.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()