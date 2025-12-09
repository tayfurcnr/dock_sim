"""
Environmental sensor models: Temperature, Humidity, Pressure, and Light.
Each model generates realistic, time-varying data with natural correlations.
"""

import math
import time
from sensor_models.noise_generators import (
    clamp, perlin_noise_1d, random_walk_step
)


class TemperatureModel:
    """
    Temperature model with day/night cycle and Perlin noise variations.
    """

    def __init__(self, config):
        """
        Initialize temperature model.

        Args:
            config: Dictionary with keys: base, daily_amplitude, noise_std, min, max
        """
        self.base_temp = config.get('base', 20.0)
        self.daily_amplitude = config.get('daily_amplitude', 8.0)
        self.noise_std = config.get('noise_std', 0.5)
        self.min_temp = config.get('min', 10.0)
        self.max_temp = config.get('max', 40.0)

        self.current_temp = self.base_temp
        self.start_time = time.time()
        self.manual_offset = 0.0  # For manual override

    def update(self, dt, current_time):
        """
        Update temperature based on time of day and noise.

        Args:
            dt: Time step in seconds
            current_time: Current simulation time in seconds since start

        Returns:
            Current temperature in Celsius
        """
        # Day/night cycle (24-hour period, coldest at 6 AM, hottest at 6 PM)
        # Phase shift: -π/2 means minimum at 6 AM (1/4 day = π/2 radians)
        cycle_value = math.sin(2 * math.pi * current_time / 86400.0 - math.pi / 2)
        daily_temp = self.base_temp + self.daily_amplitude * cycle_value

        # Add Perlin noise for natural variation
        noise = perlin_noise_1d(current_time, scale=0.01) * self.noise_std

        # Target temperature
        target_temp = daily_temp + noise + self.manual_offset

        # Rate limiting: max 0.5°C per second (realistic thermal inertia)
        max_change = 0.5 * dt
        temp_diff = target_temp - self.current_temp

        if abs(temp_diff) > max_change:
            self.current_temp += max_change if temp_diff > 0 else -max_change
        else:
            self.current_temp = target_temp

        # Clamp to physical bounds
        self.current_temp = clamp(self.current_temp, self.min_temp, self.max_temp)

        return self.current_temp

    def set_base(self, value):
        """Set base temperature for manual override."""
        if value >= 0:
            self.manual_offset = value - self.base_temp

    def get_value(self):
        """Get current temperature value."""
        return self.current_temp


class HumidityModel:
    """
    Humidity model with inverse temperature correlation and random walk.
    """

    def __init__(self, config, temperature_model):
        """
        Initialize humidity model.

        Args:
            config: Dictionary with keys: base, temp_correlation, noise_std, min, max
            temperature_model: Reference to TemperatureModel for correlation
        """
        self.base_humidity = config.get('base', 60.0)
        self.temp_correlation = config.get('temp_correlation', -0.5)
        self.noise_std = config.get('noise_std', 2.0)
        self.min_humidity = config.get('min', 20.0)
        self.max_humidity = config.get('max', 95.0)

        self.temperature_model = temperature_model
        self.current_humidity = self.base_humidity
        self.manual_offset = 0.0

    def update(self, dt, current_time):
        """
        Update humidity based on temperature correlation and random walk.

        Args:
            dt: Time step in seconds
            current_time: Current simulation time

        Returns:
            Current humidity in percent
        """
        # Get current temperature
        temp = self.temperature_model.get_value()
        temp_base = self.temperature_model.base_temp

        # Inverse correlation with temperature
        temp_effect = self.temp_correlation * (temp - temp_base)

        # Random walk for natural drift
        self.current_humidity = random_walk_step(
            self.current_humidity,
            self.noise_std,
            self.min_humidity,
            self.max_humidity
        )

        # Apply temperature correlation
        target_humidity = self.base_humidity + temp_effect + self.manual_offset
        target_humidity = clamp(target_humidity, self.min_humidity, self.max_humidity)

        # Smooth transition
        self.current_humidity = 0.9 * self.current_humidity + 0.1 * target_humidity

        return self.current_humidity

    def set_base(self, value):
        """Set base humidity for manual override."""
        if value >= 0:
            self.manual_offset = value - self.base_humidity

    def increase_for_rain(self, amount=10.0):
        """Increase humidity due to rain."""
        self.current_humidity = min(self.current_humidity + amount, self.max_humidity)

    def get_value(self):
        """Get current humidity value."""
        return self.current_humidity


class PressureModel:
    """
    Atmospheric pressure model with slow random walk.
    """

    def __init__(self, config):
        """
        Initialize pressure model.

        Args:
            config: Dictionary with keys: base, noise_std, min, max
        """
        self.base_pressure = config.get('base', 1013.25)
        self.noise_std = config.get('noise_std', 0.2)
        self.min_pressure = config.get('min', 980.0)
        self.max_pressure = config.get('max', 1030.0)

        self.current_pressure = self.base_pressure
        self.manual_offset = 0.0

    def update(self, dt, current_time):
        """
        Update pressure with slow random walk.

        Args:
            dt: Time step in seconds
            current_time: Current simulation time

        Returns:
            Current pressure in hPa
        """
        # Slow random walk: max 1 hPa per hour = 1/3600 hPa per second
        max_change_per_second = 1.0 / 3600.0
        step_sigma = max_change_per_second * dt

        self.current_pressure = random_walk_step(
            self.current_pressure + self.manual_offset,
            step_sigma,
            self.min_pressure,
            self.max_pressure
        )

        return self.current_pressure

    def set_base(self, value):
        """Set base pressure for manual override."""
        if value >= 0:
            self.manual_offset = value - self.current_pressure

    def drop_for_storm(self, amount=10.0):
        """Drop pressure for storm simulation."""
        self.current_pressure = max(self.current_pressure - amount, self.min_pressure)

    def get_value(self):
        """Get current pressure value."""
        return self.current_pressure


class LightModel:
    """
    Light intensity model with day/night cycle and cloud simulation.
    """

    def __init__(self, config):
        """
        Initialize light model.

        Args:
            config: Dictionary with keys: max_lux, night_lux, cloud_variation
        """
        self.max_lux = config.get('max_lux', 100000)
        self.night_lux = config.get('night_lux', 0)
        self.cloud_variation = config.get('cloud_variation', 0.3)

        self.current_light = 0
        self.manual_override = -1  # -1 = no override

    def update(self, dt, current_time):
        """
        Update light intensity based on solar angle and clouds.

        Args:
            dt: Time step in seconds
            current_time: Current simulation time

        Returns:
            Current light intensity in lux
        """
        if self.manual_override >= 0:
            return self.manual_override

        # Solar angle (simplified: sine wave over 24 hours)
        # 0 at midnight, max at noon
        solar_angle = math.sin(2 * math.pi * current_time / 86400.0)

        # Only positive angles (daytime)
        if solar_angle > 0:
            # Exponential makes dawn/dusk more gradual
            base_light = (solar_angle ** 2) * self.max_lux

            # Cloud simulation with Perlin noise
            # Clouds reduce light to 30-100% of clear sky value
            cloud_factor = self.cloud_variation + (1.0 - self.cloud_variation) * \
                           (0.5 + 0.5 * perlin_noise_1d(current_time, scale=0.05))

            self.current_light = int(base_light * cloud_factor)
        else:
            # Night time
            self.current_light = self.night_lux

        return self.current_light

    def set_base(self, value):
        """Set manual light override."""
        if value >= 0:
            self.manual_override = value
        else:
            self.manual_override = -1  # Disable override

    def get_value(self):
        """Get current light value."""
        return self.current_light