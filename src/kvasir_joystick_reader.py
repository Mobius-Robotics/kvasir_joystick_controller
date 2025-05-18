#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import pygame

PUBLISHED_BUTTONS = {
    "a": 0,
    "b": 1,
    "x": 2,
    "y": 3,
    "select": 6,
    "start": 7,
    "l3": 8,
    "r3": 9,
    "guide": 10,
}


class JoystickReader(Node):
    def __init__(self):
        super().__init__("kvasir_joystick_reader")
        self.joystick = None

        # Initialize pygame for joystick reading
        pygame.init()
        pygame.joystick.init()

        # Publishers for left stick X and Y axes
        self.publisher_x = self.create_publisher(Float32, "joystick/left_x", 10)
        self.publisher_y = self.create_publisher(Float32, "joystick/left_y", 10)

        # Publisher for fictitious third axis (lb and rb buttons)
        self.publisher_z = self.create_publisher(Float32, "joystick/third_axis", 10)

        self.publisher_lt = self.create_publisher(Float32, "joystick/lt", 10)
        self.publisher_rt = self.create_publisher(Float32, "joystick/rt", 10)

        self.button_publishers = {
            button: self.create_publisher(Bool, f"joystick/{button}", 10)
            for button in PUBLISHED_BUTTONS
        }

        self.timer = self.create_timer(1 / 60, self.publish_joystick_data)

        self.get_logger().info(f"Connected joysticks on start: {pygame.joystick.get_count()}")

    def publish_joystick_data(self):
        """Reads the joystick state and publishes left stick X and Y axes."""
        pygame.event.pump()  # Process events to update joystick state

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                if self.joystick is not None:
                    self.joystick.quit()
                self.joystick = None
                self.get_logger().warn(f"Disconnected from joystick")

        if self.joystick is None:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Connected to joystick: {self.joystick.get_name()}")
            else:
                return

        try:
            left_x = Float32()
            left_x.data = self.joystick.get_axis(0)  # Left stick X-axis
            self.publisher_x.publish(left_x)

            left_y = Float32()
            left_y.data = self.joystick.get_axis(1)  # Left stick Y-axis
            self.publisher_y.publish(left_y)

            lt = Float32()
            lt.data = self.joystick.get_axis(2)  # Left trigger
            self.publisher_lt.publish(lt)

            rt = Float32()
            rt.data = self.joystick.get_axis(5)  # Right trigger
            self.publisher_rt.publish(rt)

            third_axis = Float32()
            lb_pressed = self.joystick.get_button(4)  # LB button
            rb_pressed = self.joystick.get_button(5)  # RB button

            for channel, button_num in PUBLISHED_BUTTONS.items():
                publisher = self.button_publishers[channel]
                msg = Bool()
                msg.data = bool(self.joystick.get_button(button_num))
                publisher.publish(msg)

            if lb_pressed and not rb_pressed:
                third_axis.data = -1.0
            elif not lb_pressed and rb_pressed:
                third_axis.data = 1.0
            else:
                third_axis.data = 0.0

            self.publisher_z.publish(third_axis)

            self.get_logger().debug(
                f"Published: left_x={left_x.data:.2f}, left_y={left_y.data:.2f}, third_axis={third_axis.data:.2f}"
            )
            self.get_logger().debug(f"Published: lt={lt.data:.2f}, rt={rt.data:.2f}")
        except pygame.error as e:
            self.get_logger().error(f"Joystick read error: {e}")
            self.joystick = None

    def cleanup(self):
        self.get_logger().info("Shutting down joystick reader...")
        if self.joystick is not None:
            self.joystick.quit()
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    else:
        rclpy.shutdown()
    finally:
        node.cleanup()


if __name__ == "__main__":
    main()
