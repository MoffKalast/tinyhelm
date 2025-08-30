#!/usr/bin/env python3
import math
import noise

class WaveField:
    """Generates time-evolving wave heights using multi-octave Perlin noise"""

    def __init__(self, config):
        self.wave_height = config.get('wave_height', 0.5)   # meters
        self.wave_scale = config.get('wave_scale', 0.01)    # spatial scale
        self.time_scale = config.get('time_scale', 0.1)     # temporal scale
        self.octaves = config.get('octaves', 4)
        self.persistence = config.get('persistence', 0.5)
        self.lacunarity = config.get('lacunarity', 2.0)

    def get_wave_height(self, x, y, t):
        """Get wave height at world coordinates (x,y) and time t [s]."""
        return self.wave_height * noise.pnoise3(
            x * self.wave_scale,
            y * self.wave_scale,
            t * self.time_scale,
            octaves=self.octaves,
            persistence=self.persistence,
            lacunarity=self.lacunarity,
            repeatx=1024,
            repeaty=1024,
            base=0
        )

    def get_wave_normal(self, x, y, t, sample_distance=1.0):
        """Calculate wave-induced pitch/roll via gradients."""
        h_center = self.get_wave_height(x, y, t)
        h_east   = self.get_wave_height(x + sample_distance, y, t)
        h_north  = self.get_wave_height(x, y + sample_distance, t)

        dx = (h_east - h_center) / sample_distance
        dy = (h_north - h_center) / sample_distance

        pitch = math.atan2(-dx, 1.0)  # rotation about Y
        roll  = math.atan2(-dy, 1.0)  # rotation about X
        return pitch, roll
