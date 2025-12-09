"""
Noise generation functions for realistic sensor simulation.
Provides Perlin noise, Ornstein-Uhlenbeck process, and random walks.
"""

import math
import random


def clamp(value, min_val, max_val):
    """Constrain a value between min and max bounds."""
    return max(min_val, min(max_val, value))


def perlin_noise_1d(t, scale=1.0):
    """
    Simple 1D Perlin-like noise for smooth, continuous variations.

    Args:
        t: Time or position value
        scale: Frequency of the noise (higher = faster variation)

    Returns:
        Noise value in range [-1, 1]
    """
    # Use multiple octaves of sine waves for smooth, natural-looking noise
    scaled_t = t * scale

    # Multiple frequency components
    noise = 0.0
    noise += 0.5 * math.sin(scaled_t * 0.1)
    noise += 0.25 * math.sin(scaled_t * 0.37 + 1.234)
    noise += 0.125 * math.sin(scaled_t * 0.83 + 2.456)
    noise += 0.0625 * math.sin(scaled_t * 1.71 + 3.789)

    # Normalize to approximately [-1, 1]
    return noise / 0.9375


def ornstein_uhlenbeck_step(x, theta, mu, sigma, dt):
    """
    Ornstein-Uhlenbeck process step for mean-reverting random walk.
    Used for realistic wind speed simulation.

    The OU process is: dx = theta * (mu - x) * dt + sigma * dW

    Args:
        x: Current value
        theta: Mean reversion rate (higher = faster return to mean)
        mu: Mean value to revert to
        sigma: Volatility/noise amplitude
        dt: Time step

    Returns:
        New value after time step
    """
    # Mean reversion term
    drift = theta * (mu - x) * dt

    # Random diffusion term (Wiener process increment)
    diffusion = sigma * random.gauss(0, math.sqrt(dt))

    return x + drift + diffusion


def random_walk_step(x, sigma, min_val, max_val):
    """
    Constrained random walk with Gaussian steps.

    Args:
        x: Current value
        sigma: Standard deviation of the step size
        min_val: Minimum allowed value
        max_val: Maximum allowed value

    Returns:
        New value after random step, clamped to bounds
    """
    step = random.gauss(0, sigma)
    new_x = x + step
    return clamp(new_x, min_val, max_val)


def exponential_sample(mean):
    """
    Sample from exponential distribution.
    Used for rain duration modeling.

    Args:
        mean: Mean value of the distribution

    Returns:
        Random sample from exponential distribution
    """
    return random.expovariate(1.0 / mean)


def smooth_step(x, edge0=0.0, edge1=1.0):
    """
    Smooth interpolation function (similar to GLSL smoothstep).
    Useful for smooth transitions.

    Args:
        x: Input value
        edge0: Lower edge
        edge1: Upper edge

    Returns:
        Smoothly interpolated value in [0, 1]
    """
    # Clamp x to [0, 1]
    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    # Smooth Hermite interpolation
    return t * t * (3.0 - 2.0 * t)