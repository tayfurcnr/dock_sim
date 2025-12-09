#!/usr/bin/env python3
"""
Sensor Telemetry Simulator for Dock Station
Generates realistic, time-varying environmental and system telemetry data.
Publishes to standardized dock_msgs topics at 1 Hz.
"""

import rospy
import threading
import time
import uuid
from datetime import datetime, timezone
from std_msgs.msg import String

# Import dock_msgs telemetry messages
from dock_msgs.msg import (
    Lid,
    Location,
    WeatherEnv,
    WeatherWind,
    WeatherRain,
    SystemError,
    SystemNetwork,
)

# Import service definitions
from dock_station_sim.srv import (
    SetEnvironment, SetEnvironmentResponse,
    SetWind, SetWindResponse,
    TriggerWeatherEvent, TriggerWeatherEventResponse,
    GetSensorState, GetSensorStateResponse
)

# Import sensor models
from sensor_models.environment import TemperatureModel, HumidityModel, PressureModel, LightModel
from sensor_models.wind import WindModel
from sensor_models.rain import RainModel


class SensorTelemetrySimulator:
    """
    Main sensor telemetry simulator node.
    Manages all sensor models and publishes telemetry data.
    """

    def __init__(self):
        """Initialize the sensor telemetry simulator node."""
        rospy.init_node('sensor_telemetry_simulator', anonymous=False)

        # Thread lock for shared state
        self.lock = threading.Lock()

        # Load configuration from parameter server
        self.config = rospy.get_param('~sensor_simulation', {})

        # General settings
        self.publish_rate = self.config.get('publish_rate', 1.0)
        self.source_id = self.config.get('source_id', 'dock_station_001')
        self.mode = self.config.get('mode', 'realistic')
        self.time_scale = self.config.get('time_scale', 1.0)

        # Location (static)
        loc_config = self.config.get('location', {})
        self.latitude = loc_config.get('latitude', 40.7128)
        self.longitude = loc_config.get('longitude', -74.0060)
        self.altitude = loc_config.get('altitude', 10.0)

        # Network (static/simulated)
        net_config = self.config.get('network', {})
        self.ip_address = net_config.get('ip_address', '192.168.1.100')
        self.network_online = net_config.get('online', True)

        # Initialize sensor models
        env_config = self.config.get('environment', {})
        temp_config = env_config.get('temperature', {})
        humid_config = env_config.get('humidity', {})
        press_config = env_config.get('pressure', {})
        light_config = env_config.get('light', {})

        self.temperature_model = TemperatureModel(temp_config)
        self.humidity_model = HumidityModel(humid_config, self.temperature_model)
        self.pressure_model = PressureModel(press_config)
        self.light_model = LightModel(light_config)

        wind_config = self.config.get('wind', {})
        self.wind_model = WindModel(wind_config)

        rain_config = self.config.get('rain', {})
        self.rain_model = RainModel(
            rain_config,
            self.humidity_model,
            self.temperature_model,
            self.pressure_model
        )

        # Lid state (synced from dock_control_server)
        self.lid_state = Lid.STATE_CLOSED
        self.lid_percentage = 0

        # Simulation state
        self.start_time = time.time()
        self.last_update_time = self.start_time
        self.active_events = []

        # Create publishers for individual telemetry topics
        self.env_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/weather/env',
            WeatherEnv,
            queue_size=10
        )
        self.wind_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/weather/wind',
            WeatherWind,
            queue_size=10
        )
        self.rain_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/weather/rain',
            WeatherRain,
            queue_size=10
        )
        self.lid_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/lid',
            Lid,
            queue_size=10
        )
        self.location_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/location',
            Location,
            queue_size=10
        )
        self.error_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/system/error',
            SystemError,
            queue_size=10
        )
        self.network_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/system/network',
            SystemNetwork,
            queue_size=10
        )

        # Subscribe to dock control state for lid synchronization
        self.dock_state_sub = rospy.Subscriber(
            '/dock_station/state',
            String,
            self.dock_state_callback
        )

        # Create service servers for control interfaces
        self.set_env_service = rospy.Service(
            '/dock_station/sensor_sim/set_environment',
            SetEnvironment,
            self.handle_set_environment
        )
        self.set_wind_service = rospy.Service(
            '/dock_station/sensor_sim/set_wind',
            SetWind,
            self.handle_set_wind
        )
        self.trigger_event_service = rospy.Service(
            '/dock_station/sensor_sim/trigger_weather_event',
            TriggerWeatherEvent,
            self.handle_trigger_weather_event
        )
        self.get_state_service = rospy.Service(
            '/dock_station/sensor_sim/get_state',
            GetSensorState,
            self.handle_get_state
        )

        rospy.loginfo("Sensor Telemetry Simulator initialized")
        rospy.loginfo(f"  Source ID: {self.source_id}")
        rospy.loginfo(f"  Publish rate: {self.publish_rate} Hz")
        rospy.loginfo(f"  Mode: {self.mode}")
        rospy.loginfo(f"  Location: ({self.latitude}, {self.longitude}, {self.altitude}m)")
        rospy.loginfo("Services available:")
        rospy.loginfo("  /dock_station/sensor_sim/set_environment")
        rospy.loginfo("  /dock_station/sensor_sim/set_wind")
        rospy.loginfo("  /dock_station/sensor_sim/trigger_weather_event")
        rospy.loginfo("  /dock_station/sensor_sim/get_state")

    def dock_state_callback(self, msg):
        """
        Callback for dock control state updates.
        Maps dock states to lid telemetry states.

        According to DOCK_STATION_TELEMETRY.md, lid state should be:
        - "OPEN": Fully open
        - "CLOSED": Fully closed
        - "BETWEEN": In motion (opening or closing)
        """
        with self.lock:
            state_key = msg.data.strip().lower()
            state_map = {
                "open": (Lid.STATE_OPEN, 100),
                "closed": (Lid.STATE_CLOSED, 0),
                "opening": (Lid.STATE_OPENING, -1),
                "closing": (Lid.STATE_CLOSING, -1),
                "idle": (Lid.STATE_CLOSED, 0),
                "stopped": (Lid.STATE_STOPPED, -1)
            }
            self.lid_state, self.lid_percentage = state_map.get(
                state_key,
                (Lid.STATE_UNKNOWN, -1)
            )

    def get_iso_timestamp(self):
        """Generate ISO8601 timestamp string."""
        return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%fZ')

    def publish_telemetry(self):
        """
        Main publishing method called at publish_rate Hz.
        Updates all sensor models and publishes individual and aggregated messages.
        """
        current_time = time.time()
        dt = (current_time - self.last_update_time) * self.time_scale
        sim_time = (current_time - self.start_time) * self.time_scale
        self.last_update_time = current_time

        # Update all sensor models
        temperature = self.temperature_model.update(dt, sim_time)
        humidity = self.humidity_model.update(dt, sim_time)
        pressure = self.pressure_model.update(dt, sim_time)
        light = self.light_model.update(dt, sim_time)

        wind_speed, wind_direction = self.wind_model.update(dt, sim_time)
        rain_active, rain_rate = self.rain_model.update(dt, sim_time)

        # Generate timestamp (same for all messages in this cycle)
        timestamp = self.get_iso_timestamp()

        # Create and publish individual messages
        # Environmental data
        env_msg = WeatherEnv()
        env_msg.temp = temperature
        env_msg.hum = humidity
        env_msg.pres = pressure
        env_msg.light = light
        env_msg.ts = timestamp
        self.env_pub.publish(env_msg)

        # Wind data
        wind_msg = WeatherWind()
        wind_msg.speed = wind_speed
        wind_msg.dir = wind_direction
        wind_msg.ts = timestamp
        self.wind_pub.publish(wind_msg)

        # Rain data
        rain_msg = WeatherRain()
        rain_msg.active = rain_active
        rain_msg.rate = rain_rate
        rain_msg.ts = timestamp
        self.rain_pub.publish(rain_msg)

        # Lid state (from dock control)
        with self.lock:
            lid_state = self.lid_state
            lid_percentage = self.lid_percentage

        lid_msg = Lid()
        lid_msg.state = lid_state
        lid_msg.percentage = lid_percentage
        lid_msg.ts = timestamp
        self.lid_pub.publish(lid_msg)

        # Location (static)
        location_msg = Location()
        location_msg.lat = self.latitude
        location_msg.lon = self.longitude
        location_msg.alt = self.altitude
        location_msg.ts = timestamp
        self.location_pub.publish(location_msg)

        # System error (default: no error)
        error_msg = SystemError()
        error_msg.code = 0
        error_msg.msg = "OK"
        error_msg.ts = timestamp
        self.error_pub.publish(error_msg)

        # Network status
        network_msg = SystemNetwork()
        network_msg.ip = self.ip_address
        network_msg.online = self.network_online
        network_msg.ts = timestamp
        self.network_pub.publish(network_msg)

    def handle_set_environment(self, req):
        """Service handler for setting environmental parameters."""
        try:
            with self.lock:
                if req.temperature >= 0:
                    self.temperature_model.set_base(req.temperature)
                    rospy.loginfo(f"Set temperature base to {req.temperature}°C")

                if req.humidity >= 0:
                    self.humidity_model.set_base(req.humidity)
                    rospy.loginfo(f"Set humidity base to {req.humidity}%")

                if req.pressure >= 0:
                    self.pressure_model.set_base(req.pressure)
                    rospy.loginfo(f"Set pressure base to {req.pressure} hPa")

                if req.light >= 0:
                    self.light_model.set_base(req.light)
                    rospy.loginfo(f"Set light to {req.light} lux")

            return SetEnvironmentResponse(
                success=True,
                message="Environmental parameters updated"
            )

        except Exception as e:
            rospy.logerr(f"Error setting environment: {e}")
            return SetEnvironmentResponse(
                success=False,
                message=f"Error: {str(e)}"
            )

    def handle_set_wind(self, req):
        """Service handler for setting wind parameters."""
        try:
            with self.lock:
                if req.speed >= 0:
                    self.wind_model.set_speed(req.speed)
                    rospy.loginfo(f"Set wind speed to {req.speed} m/s")

                if req.direction >= 0:
                    self.wind_model.set_direction(req.direction)
                    rospy.loginfo(f"Set wind direction to {req.direction}°")

            return SetWindResponse(
                success=True,
                message="Wind parameters updated"
            )

        except Exception as e:
            rospy.logerr(f"Error setting wind: {e}")
            return SetWindResponse(
                success=False,
                message=f"Error: {str(e)}"
            )

    def handle_trigger_weather_event(self, req):
        """Service handler for triggering weather events."""
        try:
            event_id = str(uuid.uuid4())[:8]
            event_type = req.event_type.lower()

            with self.lock:
                if event_type == 'rain':
                    self.rain_model.trigger_rain(req.duration)
                    rospy.loginfo(f"Triggered rain event for {req.duration}s")

                elif event_type == 'storm':
                    # Storm: heavy rain + high wind + pressure drop
                    self.rain_model.trigger_rain(req.duration)
                    self.wind_model.trigger_gust(15.0)
                    self.pressure_model.drop_for_storm(15.0)
                    rospy.loginfo(f"Triggered storm event for {req.duration}s")

                elif event_type == 'calm':
                    # Calm: stop rain, low wind, clear skies
                    self.rain_model.stop_rain()
                    self.wind_model.trigger_calm()
                    rospy.loginfo("Triggered calm weather")

                elif event_type == 'heatwave':
                    # Heatwave: increase temperature significantly
                    current_temp = self.temperature_model.get_value()
                    self.temperature_model.set_base(current_temp + 10.0)
                    rospy.loginfo("Triggered heatwave")

                elif event_type == 'cold_front':
                    # Cold front: drop temperature, increase wind, rain possible
                    current_temp = self.temperature_model.get_value()
                    self.temperature_model.set_base(current_temp - 10.0)
                    self.wind_model.trigger_gust(10.0)
                    self.pressure_model.drop_for_storm(10.0)
                    rospy.loginfo("Triggered cold front")

                else:
                    return TriggerWeatherEventResponse(
                        success=False,
                        message=f"Unknown event type: {event_type}",
                        event_id=""
                    )

                # Track active event
                self.active_events.append(f"{event_type}_{event_id}")

            return TriggerWeatherEventResponse(
                success=True,
                message=f"Triggered {event_type} event",
                event_id=event_id
            )

        except Exception as e:
            rospy.logerr(f"Error triggering weather event: {e}")
            return TriggerWeatherEventResponse(
                success=False,
                message=f"Error: {str(e)}",
                event_id=""
            )

    def handle_get_state(self, req):
        """Service handler for querying simulation state."""
        try:
            with self.lock:
                sim_time = (time.time() - self.start_time) * self.time_scale
                events = list(self.active_events)

            return GetSensorStateResponse(
                success=True,
                mode=self.mode,
                simulation_time=sim_time,
                active_events=events
            )

        except Exception as e:
            rospy.logerr(f"Error getting state: {e}")
            return GetSensorStateResponse(
                success=False,
                mode=self.mode,
                simulation_time=0.0,
                active_events=[]
            )

    def run(self):
        """Main loop running at publish_rate Hz."""
        rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("Sensor telemetry simulator running...")

        while not rospy.is_shutdown():
            self.publish_telemetry()
            rate.sleep()


if __name__ == '__main__':
    try:
        simulator = SensorTelemetrySimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
