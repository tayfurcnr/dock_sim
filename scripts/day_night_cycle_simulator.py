#!/usr/bin/env python3
"""
Day/Night Cycle Simulator
Simulates day and night cycle by rotating the sun in Gazebo and updating lux values.
Cycle duration: ~10 minutes (configurable)

Lux values based on sun position:
  - NIGHT_DARK: < 5 lux (full darkness)
  - NIGHT_BRIGHT: 5-50 lux (moonlight/city lights)
  - DAWN: 50-300 lux (dawn/dusk)
  - DAY_CLOUDY: 300-10,000 lux (cloudy day)
  - DAY_SUNNY: > 10,000 lux (sunny day)

The node controls ONLY the lux value via the sensor_telemetry_simulator.
All other environmental values (temp, humidity, pressure) remain controlled
by the sensor_telemetry_simulator's own models.
"""

import rospy
import math
from gazebo_msgs.srv import GetLightProperties, SetLightProperties
from geometry_msgs.msg import Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Float32, String
from dock_station_sim.srv import SetEnvironment
from tf.transformations import quaternion_from_euler


class DayNightCycleSimulator:
    # Light condition thresholds (lux)
    LUX_NIGHT_DARK_MAX = 5
    LUX_NIGHT_BRIGHT_MAX = 50
    LUX_DAWN_MAX = 300
    LUX_DAY_CLOUDY_MAX = 10000

    # Light condition names
    NIGHT_DARK = "NIGHT_DARK"
    NIGHT_BRIGHT = "NIGHT_BRIGHT"
    DAWN = "DAWN"
    DAY_CLOUDY = "DAY_CLOUDY"
    DAY_SUNNY = "DAY_SUNNY"

    def __init__(self):
        rospy.init_node('day_night_cycle_simulator', anonymous=False)

        # Configuration parameters
        self.cycle_duration = rospy.get_param('~day_night_cycle/cycle_duration', 600.0)  # 10 minutes default
        self.update_rate = rospy.get_param('~day_night_cycle/update_rate', 1.0)  # Hz
        self.sun_name = rospy.get_param('~day_night_cycle/sun_name', 'sun')
        self.enable_cycle = rospy.get_param('~day_night_cycle/enable_cycle', True)
        self.start_time = rospy.get_param('~day_night_cycle/start_time', 0.5)  # 0.0 = midnight, 0.5 = noon

        # Lux value ranges for each time of day
        self.lux_night_dark = rospy.get_param('~day_night_cycle/lux_night_dark', 1.0)
        self.lux_night_bright = rospy.get_param('~day_night_cycle/lux_night_bright', 25.0)
        self.lux_dawn = rospy.get_param('~day_night_cycle/lux_dawn', 150.0)
        self.lux_day_cloudy = rospy.get_param('~day_night_cycle/lux_day_cloudy', 5000.0)
        self.lux_day_sunny = rospy.get_param('~day_night_cycle/lux_day_sunny', 50000.0)

        # State variables
        self.current_time = self.start_time  # 0.0 to 1.0 (fraction of day)
        self.current_lux = 0.0
        self.current_condition = self.NIGHT_DARK
        self.paused = not self.enable_cycle

        # Sun light properties (to preserve when updating)
        self.sun_diffuse = None
        self.sun_specular = None
        self.sun_attenuation_constant = None
        self.sun_attenuation_linear = None
        self.sun_attenuation_quadratic = None
        self.sun_cast_shadows = True

        # Publishers for monitoring
        self.lux_pub = rospy.Publisher(
            '/day_night_cycle/lux',
            Float32,
            queue_size=10
        )
        self.condition_pub = rospy.Publisher(
            '/day_night_cycle/condition',
            String,
            queue_size=10
        )
        self.time_pub = rospy.Publisher(
            '/day_night_cycle/time',
            Float32,
            queue_size=10
        )

        # Wait for sensor sim service
        rospy.loginfo("Waiting for sensor sim set_environment service...")
        try:
            rospy.wait_for_service('/dock_station/sensor_sim/set_environment', timeout=10.0)
            self.set_env_service = rospy.ServiceProxy('/dock_station/sensor_sim/set_environment', SetEnvironment)
            rospy.loginfo("Connected to sensor sim service")
        except rospy.ROSException:
            rospy.logerr("Sensor sim service not available - cannot control lux values!")
            self.set_env_service = None

        # Wait for Gazebo light property services (for rotating sun)
        rospy.loginfo("Checking for Gazebo light services...")
        try:
            rospy.wait_for_service('/gazebo/get_light_properties', timeout=3.0)
            rospy.wait_for_service('/gazebo/set_light_properties', timeout=3.0)
            self.get_light_props = rospy.ServiceProxy('/gazebo/get_light_properties', GetLightProperties)
            self.set_light_props = rospy.ServiceProxy('/gazebo/set_light_properties', SetLightProperties)
            self.gazebo_available = True
            rospy.loginfo("Gazebo light services available - sun will rotate")

            # Get initial sun properties to preserve them
            self._get_initial_sun_properties()
        except rospy.ROSException:
            rospy.logwarn("Gazebo light services not available - sun will not rotate")
            self.gazebo_available = False

        rospy.loginfo("Day/Night Cycle Simulator initialized")
        rospy.loginfo(f"  Cycle duration: {self.cycle_duration}s ({self.cycle_duration/60:.1f} min)")
        rospy.loginfo(f"  Update rate: {self.update_rate} Hz")
        rospy.loginfo(f"  Sun name: {self.sun_name}")
        rospy.loginfo(f"  Cycle enabled: {self.enable_cycle}")
        rospy.loginfo(f"  Start time: {self.start_time:.2f} (0=midnight, 0.5=noon)")

    def _get_initial_sun_properties(self):
        """Get and store initial sun light properties to preserve them."""
        try:
            response = self.get_light_props(self.sun_name)
            if response.success:
                self.sun_diffuse = response.diffuse
                self.sun_specular = ColorRGBA(0.1, 0.1, 0.1, 1.0)  # Default specular
                self.sun_attenuation_constant = response.attenuation_constant
                self.sun_attenuation_linear = response.attenuation_linear
                self.sun_attenuation_quadratic = response.attenuation_quadratic
                self.sun_cast_shadows = True  # Default for directional lights
                rospy.loginfo(f"Got initial sun properties: diffuse RGB({response.diffuse.r:.2f}, {response.diffuse.g:.2f}, {response.diffuse.b:.2f})")
            else:
                rospy.logwarn(f"Failed to get sun properties: {response.status_message}")
        except Exception as e:
            rospy.logwarn(f"Error getting initial sun properties: {e}")

    def calculate_sun_angle(self, time_fraction):
        """
        Calculate sun pitch angle based on time of day.
        time_fraction: 0.0 = midnight, 0.25 = sunrise, 0.5 = noon, 0.75 = sunset, 1.0 = midnight
        Returns angle in degrees for pitch (vertical rotation)
        """
        # Sun moves from -90° (below horizon at midnight) to +90° (zenith at noon)
        # Using linear interpolation: angle = -90° at 0 and 1, +90° at 0.5
        angle_deg = 180.0 * (time_fraction - 0.5)
        return angle_deg

    def calculate_lux(self, time_fraction):
        """
        Calculate lux value based on time of day.
        Returns lux value and condition name.
        """
        sun_angle_deg = self.calculate_sun_angle(time_fraction)

        # Calculate lux from sun elevation
        # When sun is below horizon (< 0°), it's night
        # When sun is above horizon (> 0°), it's day

        if sun_angle_deg < -18:  # Deep night
            lux = self.lux_night_dark
            condition = self.NIGHT_DARK
        elif sun_angle_deg < -6:  # Astronomical twilight
            # Transition from night dark to night bright
            progress = (sun_angle_deg + 18) / 12.0  # 0 to 1
            lux = self.lux_night_dark + progress * (self.lux_night_bright - self.lux_night_dark)
            condition = self.NIGHT_BRIGHT
        elif sun_angle_deg < 0:  # Civil twilight (dawn/dusk)
            # Transition from night bright to dawn
            progress = (sun_angle_deg + 6) / 6.0  # 0 to 1
            lux = self.lux_night_bright + progress * (self.lux_dawn - self.lux_night_bright)
            condition = self.DAWN
        elif sun_angle_deg < 15:  # Early morning/late afternoon
            # Transition from dawn to day cloudy
            progress = sun_angle_deg / 15.0  # 0 to 1
            lux = self.lux_dawn + progress * (self.lux_day_cloudy - self.lux_dawn)
            condition = self.DAY_CLOUDY
        elif sun_angle_deg < 45:  # Mid morning/afternoon
            # Transition from day cloudy to day sunny
            progress = (sun_angle_deg - 15.0) / 30.0  # 0 to 1
            lux = self.lux_day_cloudy + progress * (self.lux_day_sunny - self.lux_day_cloudy)
            condition = self.DAY_CLOUDY if lux < self.LUX_DAY_CLOUDY_MAX else self.DAY_SUNNY
        else:  # High sun (noon region)
            lux = self.lux_day_sunny
            condition = self.DAY_SUNNY

        return int(lux), condition

    def update_gazebo_sun(self, time_fraction):
        """Update sun orientation in Gazebo by setting light direction."""
        if not self.gazebo_available or self.sun_diffuse is None:
            return

        try:
            sun_angle_deg = self.calculate_sun_angle(time_fraction)

            # Clamp angle to avoid extreme values that might cause Gazebo issues
            # Keep sun between -85° and +85° to avoid singularities
            sun_angle_deg = max(-85.0, min(85.0, sun_angle_deg))
            sun_angle_rad = math.radians(sun_angle_deg)

            # Calculate direction vector from pitch angle
            # Direction points from sun toward ground
            direction = Vector3()
            direction.x = 0.0
            direction.y = math.sin(sun_angle_rad)
            direction.z = -math.cos(sun_angle_rad)

            # Ensure no component is exactly zero to avoid numerical issues
            epsilon = 1e-6
            if abs(direction.x) < epsilon:
                direction.x = epsilon
            if abs(direction.y) < epsilon:
                direction.y = epsilon if direction.y >= 0 else -epsilon
            if abs(direction.z) < epsilon:
                direction.z = -epsilon  # Prefer downward

            # Normalize direction vector to unit length
            magnitude = math.sqrt(direction.x**2 + direction.y**2 + direction.z**2)
            if magnitude < epsilon:
                # Should never happen with epsilon checks, but be safe
                direction.x = 0.0
                direction.y = 0.0
                direction.z = -1.0
                magnitude = 1.0

            direction.x /= magnitude
            direction.y /= magnitude
            direction.z /= magnitude

            # Verify all components are finite and valid
            if not (math.isfinite(direction.x) and math.isfinite(direction.y) and math.isfinite(direction.z)):
                rospy.logerr(f"Invalid direction vector: ({direction.x}, {direction.y}, {direction.z})")
                return

            # Create identity pose
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            # Ensure attenuation values are valid (positive, non-zero)
            attenuation_constant = max(self.sun_attenuation_constant, epsilon)
            attenuation_linear = max(self.sun_attenuation_linear, epsilon)
            attenuation_quadratic = max(self.sun_attenuation_quadratic, epsilon)

            # Call service with preserved properties
            response = self.set_light_props(
                light_name=self.sun_name,
                cast_shadows=self.sun_cast_shadows,
                diffuse=self.sun_diffuse,
                specular=self.sun_specular,
                attenuation_constant=attenuation_constant,
                attenuation_linear=attenuation_linear,
                attenuation_quadratic=attenuation_quadratic,
                direction=direction,
                pose=pose
            )

            if not response.success:
                rospy.logwarn_throttle(30.0, f"Failed to set sun properties: {response.status_message}")

        except Exception as e:
            rospy.logwarn_throttle(30.0, f"Failed to update sun orientation: {e}")

    def update_sensor_lux(self, lux_value):
        """Update lux value in sensor telemetry simulator."""
        if self.set_env_service is None:
            return

        try:
            # Call service with only lux parameter
            # temperature=-1, humidity=-1, pressure=-1 means "no change"
            self.set_env_service(
                temperature=-1,
                humidity=-1,
                pressure=-1,
                light=lux_value
            )
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(30.0, f"Failed to update sensor lux: {e}")

    def get_time_string(self, time_fraction):
        """Convert time fraction to HH:MM format."""
        hours = int(time_fraction * 24)
        minutes = int((time_fraction * 24 - hours) * 60)
        return f"{hours:02d}:{minutes:02d}"

    def update(self):
        """Update cycle state."""
        if not self.paused:
            # Advance time
            dt = 1.0 / self.update_rate
            time_increment = dt / self.cycle_duration
            self.current_time = (self.current_time + time_increment) % 1.0

        # Calculate current values
        self.current_lux, self.current_condition = self.calculate_lux(self.current_time)

        # Update Gazebo (optional visual effect)
        self.update_gazebo_sun(self.current_time)

        # Update sensor lux (this is the main functionality)
        self.update_sensor_lux(self.current_lux)

        # Publish monitoring values
        self.lux_pub.publish(Float32(self.current_lux))
        self.condition_pub.publish(String(self.current_condition))
        self.time_pub.publish(Float32(self.current_time))

        # Log status periodically
        time_str = self.get_time_string(self.current_time)
        sun_angle = self.calculate_sun_angle(self.current_time)
        rospy.loginfo_throttle(
            30.0,
            f"Time: {time_str} ({self.current_time:.2f}), "
            f"Sun angle: {sun_angle:.1f}°, "
            f"Condition: {self.current_condition}, "
            f"Lux: {self.current_lux}"
        )

    def run(self):
        """Main loop."""
        rate = rospy.Rate(self.update_rate)

        rospy.loginfo("Day/Night cycle simulator running...")
        rospy.loginfo(f"Starting at time: {self.get_time_string(self.current_time)}")

        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == '__main__':
    try:
        simulator = DayNightCycleSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
