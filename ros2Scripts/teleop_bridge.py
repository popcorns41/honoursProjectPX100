#!/usr/bin/env python3
import serial, math, time
import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
        self.pub = self.create_publisher(JointGroupCommand,
                                         '/px100/commands/joint_group', 10)

        # direction multipliers (flip +1/-1 as needed)
        self.directions = [1, 1, -1, 1]

        self.offsets = [0.0] * 4
        self.last_valid = [0.0] * 4
        self.calib_samples = []
        self.calibrated = False

        self.calib_duration = 2.0
        self.start_time = time.time()

        self.timer = self.create_timer(0.005, self.update)
        self.get_logger().info("TeleopBridge: hold both arms in SLEEP pose for calibration...")

    # --------------------------------------------------------
    def wrap_angle(self, a):
        if a > math.pi:
            a -= 2 * math.pi
        elif a < -math.pi:
            a += 2 * math.pi
        return a

    # --------------------------------------------------------
    def update(self):
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return
            vals = [float(v) for v in line.split(',')]
        except Exception:
            return

        # pad or trim to 4 values
        while len(vals) < 4:
            vals.append(0.0)
        vals = vals[:4]

        # replace zeros with last valid readings
        for i in range(4):
            if abs(vals[i]) < 1e-5:
                vals[i] = self.last_valid[i]
            else:
                self.last_valid[i] = vals[i]

        # calibration phase
        if not self.calibrated:
            if time.time() - self.start_time < self.calib_duration:
                self.calib_samples.append(vals)
                return
            else:
                if self.calib_samples:
                    self.offsets = [sum(c) / len(c) for c in zip(*self.calib_samples)]
                self.calibrated = True
                self.get_logger().info(f"Calibration complete — offsets: {self.offsets}")
                self.get_logger().info("Teleop active! Move your mini-arm.")
                return

        # streaming phase
        cmd = []
        for i in range(4):
            delta = self.wrap_angle(vals[i] - self.offsets[i])
            cmd.append(self.directions[i] * delta)

        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = cmd
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()