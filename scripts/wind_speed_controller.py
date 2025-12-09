#!/usr/bin/env python3
"""
Wind Speed Indicator Controller
Rotates the ruzgar_hiz joint based on real-time wind speed data.

Subscribes to: /dock/rx/devices/telemetry/weather/wind (dock_msgs/WeatherWind)
Publishes to: /dock_station/ruzgar_hizj_position_controller/command (std_msgs/Float64)

The controller applies a linear mapping from wind speed (m/s) to angular velocity (rad/s)
and continuously increments the joint position to simulate rotation.
"""

import rospy
from std_msgs.msg import Float64
from dock_msgs.msg import WeatherWind
import math


class WindSpeedController:
    """
    Controller that maps wind speed data to wind indicator rotation.
    Uses position controller with continuous position increments to simulate velocity.
    """

    def __init__(self):
        rospy.init_node('wind_speed_controller')

        # Load configuration parameters
        self.velocity_scale = rospy.get_param('~wind_speed_indicator/velocity_scale_factor', 0.5)
        self.max_velocity = rospy.get_param('~wind_speed_indicator/max_velocity', 10.0)
        self.min_velocity = rospy.get_param('~wind_speed_indicator/min_velocity', 0.0)
        self.update_rate = rospy.get_param('~wind_speed_indicator/update_rate', 10)

        # Current wind speed (updated by subscriber callback)
        self.current_wind_speed = 0.0

        # Current position tracker (continuous joint, can wrap around)
        self.current_position = 0.0

        # Publisher for position command to the joint controller
        self.position_pub = rospy.Publisher(
            '/dock_station/ruzgar_hizj_position_controller/command',
            Float64,
            queue_size=10
        )

        # Subscriber to wind data topic
        self.wind_sub = rospy.Subscriber(
            '/dock/rx/devices/telemetry/weather/wind',
            WeatherWind,
            self.wind_callback
        )

        rospy.loginfo("Wind speed indicator controller started")
        rospy.loginfo(f"Scale factor: {self.velocity_scale}, Max velocity: {self.max_velocity} rad/s")
        rospy.loginfo(f"Update rate: {self.update_rate} Hz")

    def wind_callback(self, msg):
        """
        Callback for wind data updates.

        Args:
            msg (WeatherWind): Wind data message containing speed, direction, and timestamp
        """
        self.current_wind_speed = msg.speed

    def calculate_velocity(self):
        """
        Calculate angular velocity from current wind speed.

        Applies linear scaling and enforces min/max limits.

        Returns:
            float: Target angular velocity in rad/s
        """
        # Linear mapping: angular_velocity = wind_speed * scale_factor
        velocity = self.current_wind_speed * self.velocity_scale

        # Apply velocity limits
        velocity = max(self.min_velocity, min(velocity, self.max_velocity))

        return velocity

    def run(self):
        """
        Main control loop.

        Continuously updates position based on velocity at the configured update rate.
        """
        rate = rospy.Rate(self.update_rate)
        dt = 1.0 / self.update_rate  # Time step in seconds

        rospy.loginfo("Wind speed controller loop started")

        while not rospy.is_shutdown():
            # Calculate target velocity from current wind speed
            velocity = self.calculate_velocity()

            # Increment position based on velocity (position += velocity * dt)
            position_increment = velocity * dt
            self.current_position += position_increment

            # Normalize to keep within reasonable bounds (optional for continuous joint)
            # This prevents the position value from growing too large
            self.current_position = self.current_position % (2 * math.pi)

            # Publish position command to controller
            self.position_pub.publish(Float64(self.current_position))

            # Sleep to maintain update rate
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = WindSpeedController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
