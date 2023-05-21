#!/usr/bin/env python3
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import Bool

from solution import publish_leds


class LedPublisherNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LedPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.log("Initializing ...")

        # Determine what Duckiebot the node is running on
        self.veh = rospy.get_namespace().strip("/")

        # Start ping subscriber
        rospy.Subscriber(f"/{self.veh}/led_node/update", Bool, self.fade_through_colors, queue_size=1)

        # LED publisher
        self.pub_leds = rospy.Publisher(
            f"/{self.veh}/led_emitter_node/led_pattern",
            LEDPattern,
            queue_size=1
        )
        self.log("Initialized.")

    def fade_through_colors(self, msg):
        self.log("Publishing to the LEDs ...")
        for color in ["red", "green", "blue", "white"]:
            for intensity in [0.25, 0.5, 0.75, 1.0]:
                message: LEDPattern = publish_leds.set_leds_color(color, intensity)
                self.pub_leds.publish(message)
                rospy.sleep(3.)
        self.log("Done!")

    def on_shutdown(self):
        super(LedPublisherNode, self).on_shutdown()


if __name__ == "__main__":
    # Initialize the node
    leds_publisher_node = LedPublisherNode(node_name="led_publisher_node")
    # Keep it spinning
    rospy.spin()




