import rclpy
from rclpy.node import Node
import qi
import argparse
import sys
import time

from std_srvs.srv import SetBool
from std_msgs.msg import Float32
from naoqi_utilities_msgs.msg import LedParameters

class NaoqiMiscellaneousNode(Node):
    """
    ROS2 Node to manage miscellaneous functionalities of a NAO robot.
    """
    def __init__(self, session, robot_url=None):
        """
        Initializes the node, NAOqi service clients, and ROS2 services and publishers.
        """
        super().__init__('naoqi_miscellaneous_node')
        self.get_logger().info("Initializing NaoqiMiscellaneousNode...")
        self.session = session
        self.robot_url = robot_url
        self.battery_warning_count = 0

        # --- NAOqi Service Clients ---
        try:
            self._refresh_naoqi_services()
            self.get_logger().info("NAOqi service clients obtained successfully.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to NAOqi services: {e}")
            # Exit if essential services cannot be obtained.
            sys.exit(1)

        # --- ROS2 Services ---
        # ALAutonomousLife
        self.set_autonomous_state_service = self.create_service(
            SetBool,
            'set_autonomous_state',
            self.set_autonomous_state_callback
        )

        # ALBasicAwareness
        self.toggle_awareness_service = self.create_service(
            SetBool,
            'toggle_awareness',
            self.toggle_awareness_callback
        )

        # ALAutonomousBlinking
        self.toggle_blinking_service = self.create_service(
            SetBool,
            'toggle_blinking',
            self.toggle_blinking_callback
        )

        # --- ROS2 Publishers and Subscribers ---
        # ALBatteryService
        self.battery_publisher = self.create_publisher(Float32, 'battery_percentage', 10)
        self.battery_timer = self.create_timer(5.0, self.publish_battery_percentage) # Publish every 5 seconds

        # ALLeds
        self.leds_subscriber = self.create_subscription(
            LedParameters,
            'set_leds',
            self.leds_callback,
            10
        )

        self.get_logger().info("Miscellaneous functionalities node is ready.")

    def _ensure_session_connected(self):
        if self.session.isConnected():
            return True
        if not self.robot_url:
            return False
        self.get_logger().warn("NAOqi session is disconnected. Attempting to reconnect...")
        try:
            self.session.close()
        except Exception:
            pass
        try:
            self.session.connect(self.robot_url)
            self.get_logger().info("NAOqi session reconnected.")
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not reconnect NAOqi session: {e}")
            return False

    def _service(self, name):
        if not self._ensure_session_connected():
            raise RuntimeError("NAOqi session is not connected")
        return self.session.service(name)

    def _refresh_naoqi_services(self):
        self.al_autonomous_life = self._service("ALAutonomousLife")
        self.al_basic_awareness = self._service("ALBasicAwareness")
        self.al_battery = self._service("ALBattery")
        self.al_autonomous_blinking = self._service("ALAutonomousBlinking")
        self.al_robot_posture = self._service("ALRobotPosture")
        self.al_leds = self._service("ALLeds")

    def leds_callback(self, msg):
        """
        Callback to set the color of an LED or group of LEDs.
        """
        try:
            # Normalize color values from 0-255 to 0.0-1.0
            red = msg.red / 255.0
            green = msg.green / 255.0
            blue = msg.blue / 255.0
            
            self.get_logger().info(f"Setting LED '{msg.name}' to color ({red:.2f}, {green:.2f}, {blue:.2f}) over {msg.duration}s.")
            
            # The fadeRGB function is non-blocking
            self.al_leds.fadeRGB(msg.name, red, green, blue, msg.duration)
        except Exception as e:
            self.get_logger().error(f"Could not set LED '{msg.name}': {e}")

    def set_autonomous_state_callback(self, request, response):
        """
        Callback to enable or disable the autonomous life mode.
        """
        try:
            state = 'enabling' if request.data else 'disabling'
            self.get_logger().info(f"Request to {state} autonomous life.")
            self.al_autonomous_life.setAutonomousAbilityEnabled("All", request.data)
            if request.data:
                self.al_autonomous_life.setState("interactive")
                response.message = "Autonomous life enabled."
            else:
                if self.al_autonomous_life.getState() != "disabled":
                    self.al_autonomous_life.setState("disabled")
                    time.sleep(2)  # To avoid robot arm bug
                    self.al_robot_posture.goToPosture("Stand", 0.5)
                response.message = "Autonomous life disabled."
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Error changing autonomous life state: {e}"
            self.get_logger().error(response.message)
        return response

    def toggle_awareness_callback(self, request, response):
        """
        Callback to enable or disable basic awareness.
        """
        try:
            if request.data:
                self.al_basic_awareness.stopAwareness()
                self.al_basic_awareness.startAwareness()
                self.al_basic_awareness.pauseAwareness()
                self.al_basic_awareness.resumeAwareness()
                self.get_logger().info(f"Request to set basic awareness to {request.data}.")
            else:
                self.al_basic_awareness.startAwareness()
                self.al_basic_awareness.resumeAwareness()
                self.al_basic_awareness.pauseAwareness()
                self.al_basic_awareness.stopAwareness()
                self.get_logger().info(f"Request to set basic awareness to {request.data}.")
            response.success = True
            response.message = f"Basic awareness {request.data}."
        except Exception as e:
            response.success = False
            response.message = f"Error changing basic awareness state: {e}"
            self.get_logger().error(response.message)
        return response

    def toggle_blinking_callback(self, request, response):
        """
        Callback to enable or disable autonomous blinking.
        """
        try:
            state = 'enabled' if request.data else 'disabled'
            self.get_logger().info(f"Request to set autonomous blinking to {state}.")
            self.al_autonomous_blinking.setEnabled(request.data)
            response.success = True
            response.message = f"Autonomous blinking {state}."
        except Exception as e:
            response.success = False
            response.message = f"Error changing blinking state: {e}"
            self.get_logger().error(response.message)
        return response

    def publish_battery_percentage(self):
        """
        Gets the battery charge and publishes it as a percentage.
        """
        try:
            charge = self._read_battery_charge()
            msg = Float32()
            msg.data = float(charge)
            self.battery_publisher.publish(msg)
        except Exception as e:
            self.battery_warning_count += 1
            if self.battery_warning_count == 1 or self.battery_warning_count % 12 == 0:
                self.get_logger().warn(f"Could not get battery state: {e}")

    def _read_battery_charge(self):
        try:
            charge = self.al_battery.getBatteryCharge()
            self.battery_warning_count = 0
            return charge
        except Exception as first_error:
            self.get_logger().warn(f"Battery service read failed. Refreshing NAOqi service proxy: {first_error}")
            self.al_battery = self._service("ALBattery")
            charge = self.al_battery.getBatteryCharge()
            self.battery_warning_count = 0
            return charge


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On Robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    # Use rclpy.utilities.remove_ros_args to avoid conflicts with ROS2 arguments
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])

    session = qi.Session()
    robot_url = f"tcp://{parsed_args.ip}:{parsed_args.port}"
    try:
        session.connect(robot_url)
    except RuntimeError:
        print(f"Can't connect to Naoqi at ip \"{parsed_args.ip}\" on port {parsed_args.port}.\n"
              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    naoqi_miscellaneous_node = NaoqiMiscellaneousNode(session, robot_url)

    try:
        rclpy.spin(naoqi_miscellaneous_node)
    except KeyboardInterrupt:
        print("Closing the miscellaneous functionalities node.")
    finally:
        naoqi_miscellaneous_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()