"""
Wind model using Ornstein-Uhlenbeck process for realistic variations.
"""

import random
from sensor_models.noise_generators import clamp, ornstein_uhlenbeck_step


class WindModel:
    """
    Wind speed and direction model with mean-reverting behavior.
    Speed uses Ornstein-Uhlenbeck process, direction uses random walk.
    """

    def __init__(self, config):
        """
        Initialize wind model.

        Args:
            config: Dictionary with keys:
                - mean_speed: Average wind speed (m/s)
                - reversion_rate: OU theta parameter
                - speed_std: OU sigma parameter
                - max_speed: Maximum allowed wind speed
                - direction_std: Standard deviation for direction changes
        """
        self.mean_speed = config.get('mean_speed', 5.0)
        self.reversion_rate = config.get('reversion_rate', 0.1)
        self.speed_std = config.get('speed_std', 2.0)
        self.max_speed = config.get('max_speed', 25.0)
        self.direction_std = config.get('direction_std', 5.0)

        # Initialize with mean values
        self.current_speed = self.mean_speed
        self.current_direction = random.uniform(0, 360)

        # For manual override
        self.manual_speed_offset = 0.0
        self.manual_direction_offset = 0.0

    def update(self, dt, current_time):
        """
        Update wind speed and direction.

        Args:
            dt: Time step in seconds
            current_time: Current simulation time

        Returns:
            Tuple of (speed in m/s, direction in degrees)
        """
        # Update speed using Ornstein-Uhlenbeck process
        # Mean-reverts to mean_speed with natural fluctuations
        effective_mean = self.mean_speed + self.manual_speed_offset

        self.current_speed = ornstein_uhlenbeck_step(
            self.current_speed,
            self.reversion_rate,
            effective_mean,
            self.speed_std,
            dt
        )

        # Clamp speed to valid range [0, max_speed]
        self.current_speed = clamp(self.current_speed, 0.0, self.max_speed)

        # Update direction with random walk
        direction_change = random.gauss(0, self.direction_std)
        self.current_direction += direction_change + self.manual_direction_offset

        # Wrap direction to [0, 360)
        self.current_direction = self.current_direction % 360.0

        return self.current_speed, self.current_direction

    def set_speed(self, value):
        """Set target wind speed."""
        if value >= 0:
            self.manual_speed_offset = value - self.mean_speed

    def set_direction(self, value):
        """Set wind direction."""
        if 0 <= value < 360:
            self.manual_direction_offset = value - self.current_direction
            # Apply immediately and reset offset
            self.current_direction = value
            self.manual_direction_offset = 0.0

    def trigger_gust(self, intensity=10.0):
        """Trigger a wind gust by temporarily increasing speed."""
        self.current_speed = min(self.current_speed + intensity, self.max_speed)

    def trigger_calm(self):
        """Trigger calm conditions (low wind)."""
        self.manual_speed_offset = -self.mean_speed + 1.0  # Reduce to ~1 m/s

    def get_values(self):
        """Get current wind speed and direction."""
        return self.current_speed, self.current_direction