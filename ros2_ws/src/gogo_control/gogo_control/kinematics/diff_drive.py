from math import pi


class DiffDriveKinematics:
    def __init__(self, wheel_separation, wheel_radius, encoder_cpr, max_tps):
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.encoder_cpr = encoder_cpr
        self.max_tps = max_tps

        # Precompute conversion factor
        self.ticks_per_meter = encoder_cpr / (2 * pi * wheel_radius)

    def twist_to_tps(self, v: float, w: float):
        """
        Convert linear velocity (v) and angular velocity (w)
        into left/right wheel ticks per second.
        """

        # Differential drive kinematics
        half_wL = w * self.wheel_separation / 2.0
        v_left = v - half_wL
        v_right = v + half_wL

        # Convert m/s to ticks/sec
        left_tps = v_left * self.ticks_per_meter
        right_tps = v_right * self.ticks_per_meter

        # Arc-preserving proportional clamp
        max_mag = max(abs(left_tps), abs(right_tps))
        if max_mag > self.max_tps:
            scale = self.max_tps / max_mag
            left_tps *= scale
            right_tps *= scale

        return int(left_tps), int(right_tps)
