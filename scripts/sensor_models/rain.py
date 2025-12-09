"""
Rain model with probabilistic state machine and intensity sampling.
"""

import random
import time
from sensor_models.noise_generators import exponential_sample


class RainModel:
    """
    Rain simulation with state machine: no_rain <-> raining.
    Includes realistic duration and intensity distributions.
    """

    def __init__(self, config, humidity_model=None, temperature_model=None, pressure_model=None):
        """
        Initialize rain model.

        Args:
            config: Dictionary with keys:
                - transition_probability: Chance to start rain per update
                - mean_duration: Average rain duration (seconds)
                - light_intensity: [min, max] for light rain (mm/hr)
                - moderate_intensity: [min, max] for moderate rain
                - heavy_intensity: [min, max] for heavy rain
                - light_probability: Probability of light rain
                - moderate_probability: Probability of moderate rain
                - heavy_probability: Probability of heavy rain
            humidity_model: Reference to HumidityModel (optional)
            temperature_model: Reference to TemperatureModel (optional)
            pressure_model: Reference to PressureModel (optional)
        """
        self.transition_prob = config.get('transition_probability', 0.001)
        self.mean_duration = config.get('mean_duration', 1800)

        # Intensity ranges
        self.light_intensity = config.get('light_intensity', [0.5, 2.5])
        self.moderate_intensity = config.get('moderate_intensity', [2.5, 10.0])
        self.heavy_intensity = config.get('heavy_intensity', [10.0, 50.0])

        # Intensity probabilities
        self.light_prob = config.get('light_probability', 0.70)
        self.moderate_prob = config.get('moderate_probability', 0.20)
        self.heavy_prob = config.get('heavy_probability', 0.10)

        # State
        self.is_raining = False
        self.rain_rate = 0.0
        self.rain_start_time = 0.0
        self.rain_duration = 0.0

        # References to other models for weather effects
        self.humidity_model = humidity_model
        self.temperature_model = temperature_model
        self.pressure_model = pressure_model

        # Manual control
        self.forced_rain = False
        self.forced_duration = 0.0

    def update(self, dt, current_time):
        """
        Update rain state and intensity.

        Args:
            dt: Time step in seconds
            current_time: Current simulation time

        Returns:
            Tuple of (active: bool, rate: float in mm/hr)
        """
        if self.forced_rain:
            # Manual rain trigger
            if current_time - self.rain_start_time >= self.forced_duration:
                self._stop_rain()
            return self.is_raining, self.rain_rate

        if self.is_raining:
            # Check if rain should stop
            elapsed = current_time - self.rain_start_time
            if elapsed >= self.rain_duration:
                self._stop_rain()
        else:
            # Check if rain should start (probabilistic)
            if random.random() < self.transition_prob:
                self._start_rain(current_time)

        return self.is_raining, self.rain_rate

    def _start_rain(self, current_time):
        """Start a rain event with random duration and intensity."""
        self.is_raining = True
        self.rain_start_time = current_time

        # Sample duration from exponential distribution
        self.rain_duration = exponential_sample(self.mean_duration)

        # Sample intensity based on probability distribution
        rand_val = random.random()

        if rand_val < self.light_prob:
            # Light rain
            self.rain_rate = random.uniform(*self.light_intensity)
        elif rand_val < self.light_prob + self.moderate_prob:
            # Moderate rain
            self.rain_rate = random.uniform(*self.moderate_intensity)
        else:
            # Heavy rain
            self.rain_rate = random.uniform(*self.heavy_intensity)

        # Apply weather effects
        self._apply_rain_effects()

    def _stop_rain(self):
        """Stop the rain event."""
        self.is_raining = False
        self.rain_rate = 0.0
        self.forced_rain = False

    def _apply_rain_effects(self):
        """Apply effects of rain on other weather parameters."""
        # Increase humidity
        if self.humidity_model:
            self.humidity_model.increase_for_rain(10.0)

        # Drop temperature (evaporative cooling)
        if self.temperature_model:
            temp_drop = random.uniform(2.0, 5.0)
            current_temp = self.temperature_model.get_value()
            self.temperature_model.current_temp = current_temp - temp_drop

        # Drop pressure if heavy rain (storm system)
        if self.pressure_model and self.rain_rate > 10.0:
            self.pressure_model.drop_for_storm(5.0)

    def trigger_rain(self, duration=300.0):
        """
        Manually trigger rain event.

        Args:
            duration: Duration in seconds (0 = use default sampling)
        """
        self.forced_rain = True
        self.is_raining = True
        self.rain_start_time = time.time()

        if duration > 0:
            self.forced_duration = duration
            self.rain_duration = duration
        else:
            self.forced_duration = exponential_sample(self.mean_duration)
            self.rain_duration = self.forced_duration

        # Random intensity
        self.rain_rate = random.uniform(*self.moderate_intensity)

        self._apply_rain_effects()

    def stop_rain(self):
        """Manually stop rain."""
        self._stop_rain()

    def get_state(self):
        """Get current rain state."""
        return self.is_raining, self.rain_rate