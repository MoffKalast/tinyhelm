#!/usr/bin/env python3
import math

GOLDEN_ANGLE = 2.399963229728653

def drift(t, seed=0.0):
    return (math.sin(t + seed) + 0.5 * math.sin(2.137 * t + 1.7 + seed) + 0.25 * math.sin(5.319 * t + 4.1 + seed)) / 1.75

class WaveField:

    def __init__(self, config):
        self.wave_height = config.get('wave_height', 0.5)
        self.wave_scale = config.get('wave_scale', 0.01)
        self.time_scale = config.get('time_scale', 0.1)
        self.octaves = config.get('octaves', 4)
        self.persistence = config.get('persistence', 0.5)
        self.lacunarity = config.get('lacunarity', 2.0)

        self.components = []
        total = 0.0

        for i in range(max(1, int(self.octaves))):
            amplitude = self.persistence ** i
            frequency = self.lacunarity ** i
            angle = i * GOLDEN_ANGLE
            self.components.append((amplitude, frequency, math.cos(angle), math.sin(angle), 0.7 * i))
            total += amplitude

        self.normalizer = self.wave_height / total

    def get_wave_height(self, x, y, t):
        height = 0.0

        for amplitude, frequency, dx, dy, phase in self.components:
            k = 2.0 * math.pi * self.wave_scale * frequency
            omega = 2.0 * math.pi * self.time_scale * frequency
            height += amplitude * math.sin(k * (x * dx + y * dy) + omega * t + phase)

        return self.normalizer * height

    def get_wave_normal(self, x, y, t, sample_distance=1.0):
        h_center = self.get_wave_height(x, y, t)
        h_east = self.get_wave_height(x + sample_distance, y, t)
        h_north = self.get_wave_height(x, y + sample_distance, t)

        dx = (h_east - h_center) / sample_distance
        dy = (h_north - h_center) / sample_distance

        pitch = math.atan2(-dx, 1.0)
        roll = math.atan2(-dy, 1.0)
        return pitch, roll
