#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   7    8    9
   4    5    6
   1    2    3

q/w : increase/decrease max speeds by 0.04 and 0.2
a/s : increase/decrease only linear speed by 0.04
z/x : increase/decrease only angular speed by 0.2

CTRL-C to quit
"""

moveBindings = {
    '2': (-1, 0, 0, 0),
    '1': (-1, 0, 0, -1),
    '4': (0, 0, 0, 1),
    '6': (0, 0, 0, -1),
    '3': (-1, 0, 0, 1),
    '8': (1, 0, 0, 0),
    '7': (1, 0, 0, 1),
    '9': (1, 0, 0, -1),
}

speedBindings = {
    'q': (0.04, 0.2),
    'w': (-0.04, -0.2),
    'a': (0.04, 0),
    's': (-0.04, 0),
    'z': (0, 0.2),
    'x': (0, -0.2),
}


class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        # Parameters
        self.speed = self.declare_parameter("speed", 0.04).value
        self.turn = self.declare_parameter("turn", 0.2).value
        self.speed_limit = self.declare_parameter("speed_limit", 2.04).value
        self.turn_limit = self.declare_parameter("turn_limit", 10.0).value
        self.key_timeout = self.declare_parameter("key_timeout", 0.5).value
        self.stamped = self.declare_parameter("stamped", False).value
        self.twist_frame = self.declare_parameter("frame_id", "").value

        # Publisher
        self.publisher = self.create_publisher(TwistStamped if self.stamped else Twist, 'cmd_vel', 10)

        # Initialize variables
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0

        # Start publishing thread
        self.publish_thread = PublishThread(self.publisher, self.stamped, self.twist_frame)
        self.publish_thread.start()

    def run(self):
        settings = save_terminal_settings()
        try:
            print(msg)
            print(self.vels(self.speed, self.turn))
            while True:
                key = get_key(settings, self.key_timeout)
                if key in moveBindings:
                    self.x, self.y, self.z, self.th = moveBindings[key]
                elif key in speedBindings:
                    self.speed = min(self.speed_limit, max(self.speed + speedBindings[key][0], 0.0))
                    self.turn = min(self.turn_limit, max(self.turn + speedBindings[key][1], 0.0))
                    print(self.vels(self.speed, self.turn))
                else:
                    if key == '' and self.x == 0 and self.y == 0 and self.z == 0 and self.th == 0:
                        continue
                    self.x, self.y, self.z, self.th = 0, 0, 0, 0
                    if key == '\x03':  # CTRL-C
                        break

                self.publish_thread.update(self.x, self.y, self.z, self.th, self.speed, self.turn)
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self.publish_thread.stop()
            restore_terminal_settings(settings)

    def vels(self, speed, turn):
        return f"currently:\tLinear {speed:.2f} m/s\tAngular {turn:.2f} rad/s"


class PublishThread(threading.Thread):
    def __init__(self, publisher, stamped, twist_frame):
        super().__init__()
        self.publisher = publisher
        self.stamped = stamped
        self.twist_frame = twist_frame
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.done = False
        self.condition = threading.Condition()

    def update(self, x, y, z, th, speed, turn):
        with self.condition:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.th = float(th)
            self.speed = float(speed)
            self.turn = float(turn)
            self.condition.notify()

    def run(self):
        while not self.done:
            with self.condition:
                self.condition.wait()
                msg = TwistStamped() if self.stamped else Twist()
                if self.stamped:
                    msg.header.stamp = rclpy.time.Time().to_msg()
                    msg.header.frame_id = self.twist_frame

                msg.linear.x = self.x * self.speed
                msg.linear.y = self.y * self.speed
                msg.linear.z = self.z
                msg.angular.z = self.th * self.turn

                self.publisher.publish(msg)

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)


def get_key(settings, timeout):
    if sys.platform == 'win32':
        return msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = TeleopTwistKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

