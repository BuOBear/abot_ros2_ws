"""Physical open-loop board service. Opening/closing the port sends no pose."""
import rclpy
import serial
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from jy_real_interfaces.srv import SetServoAngle
from .servo import load_profile, ServoTransport


class Arm(Node):
    def __init__(self):
        super().__init__('servo_arm')
        for name, default in {'port': '/dev/jy_arm', 'profile': '',
                              'minimum_interval_sec': 0.2}.items():
            self.declare_parameter(name, default)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('physical arm requires use_sim_time=false')
        profile = load_profile(self.get_parameter('profile').value)
        interval = self.get_parameter('minimum_interval_sec').value
        # Validate before opening the device, including the rate limit.
        self.transport = ServoTransport(None, profile, interval)
        self.port = serial.Serial(self.get_parameter('port').value, baudrate=9600,
                                  bytesize=8, parity='N', stopbits=1, timeout=0,
                                  write_timeout=0.2, exclusive=True)
        self.transport.port = self.port
        self.last_result = 'port open; no command sent; position feedback unavailable'
        self.service = self.create_service(SetServoAngle, '/arm/set_servo_angle', self.command)
        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.diagnostic)

    def command(self, request, response):
        try:
            self.transport.send(request.actuator, request.board_angle_deg)
            response.sent_unverified = True
            response.message = 'Serial bytes written; board receipt and physical position UNVERIFIED'
        except Exception as exc:
            response.sent_unverified = False
            response.message = str(exc)
        self.last_result = response.message
        return response

    def diagnostic(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='jy/servo_arm', hardware_id=self.port.port)
        status.level = DiagnosticStatus.ERROR if self.transport.fault else DiagnosticStatus.WARN
        status.message = self.last_result
        message.status = [status]
        self.publisher.publish(message)

    def destroy_node(self):
        self.port.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Arm()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
