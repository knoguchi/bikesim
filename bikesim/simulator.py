import sys
from math import sin, cos, radians, degrees

import matplotlib.pyplot as plt
from datetime import timedelta

from bikesim import Vector
from bikesim.constants import u

ordinal = lambda n: "%d%s" % (n, "tsnrhtdd"[(n / 10 % 10 != 1) * (n % 10 < 4) * n % 10::4])


class Simulator:

    def __init__(self, bike, rider, dt=0.01):
        self.dt = dt  # simulation time step in second
        self.bike = bike
        self.rider = rider
        self.rider.ride_on(bike)
        self.t = 0
        self.cadence = 0
        self.setup_canvas()

    def run(self):
        prev_sec = None
        while True:
            if prev_sec is None:
                prev_sec = int(self.t)
            vector = self.rider.run(self.dt, self.cadence, u)

            # world coordinate
            wp = self.bike.get_world_position()

            # rotate the vector based on the current angle in the world coordinate
            x2 = vector.x * cos(wp.angle) - vector.y * sin(wp.angle)
            y2 = vector.x * sin(wp.angle) + vector.y * cos(wp.angle)

            # new position in the world coordinate is current + rotated vector
            new_pos = Vector(wp.x + x2, wp.y + y2, radians(degrees(vector.angle + wp.angle) % 360), vector.lean_angle)
            self.bike.set_world_position(new_pos)

            self.bike_axes.clear()
            self.bike_axes.axis([-1, 1, -1, 1])
            # self.bike_axes.arrow(0, 0, vector.x, vector.y, head_width=0.05, head_length=0.1, fc='k', ec='k')
            self.bike_axes.arrow(0, 0, x2, y2, head_width=0.05, head_length=0.1, fc='k', ec='k')

            self.bike_axes.text(0, 0, self.bike.steer_angle())

            # self.world_axes.clear()
            # self.world_axes.axis([-100, 100, -100, 100])
            # self.world_axes.arrow(wp.x, wp.y, x2, y2, head_width=0.05, head_length=0.1, fc='k', ec='k')
            self.world_axes.scatter(new_pos.x, new_pos.y)

            self.t += self.dt
            plt.pause(0.01)

            if prev_sec != int(self.t):
                print("\033[2J\033[1;1H")
                print("========== simulation  ==========")
                print(timedelta(seconds=self.t))
                print("============= rider =============")
                print(self.rider_stats())
                print("")
                print("============= bike ==============")
                print(self.bike_stats())
                prev_sec = int(self.t)
                # print(self.bike)

    def setup_canvas(self):
        def press(event):
            sys.stdout.flush()
            if event.key == 'left':
                self.bike.set_steer(self.bike.steer_angle() + 1)
            elif event.key == ' ':
                self.bike.set_steer(0)
            elif event.key == 'right':
                self.bike.set_steer(self.bike.steer_angle() - 1)
            elif event.key == 'up':
                self.cadence += 10
            elif event.key == 'down':
                self.cadence -= 10
            elif event.key == '1':
                self.bike.shift_down_front()
            elif event.key == '2':
                self.bike.shift_up_front()
            elif event.key == '0':
                self.bike.shift_down_rear()
            elif event.key == '9':
                self.bike.shift_up_rear()

        fig = plt.figure(figsize=(7, 4))
        fig.canvas.mpl_connect('key_press_event', press)

        self.world_axes = fig.add_subplot(121)  # left box
        self.world_axes.set_aspect(1)

        self.bike_axes = fig.add_subplot(122)  # right box
        fig.tight_layout()

    def bike_stats(self):
        keys = [
            ("power", "Power", ",", "W", int),
            ("cadence", "Cadence", ",", "rpm", int),
            ("v", "Speed", ".2f", "km/h", lambda x: x * 3600 / 1000),
            (None, None, None, None, None),
            ("accelaration", "Accel", ".2f", "m/s^2", float),
            ("bottom_bracket_torque", "BB torque", ".2f", "Nm", float),
            ("chain_tension", "Chain tension", ".2f", "N", float),
            ("rear_hub_torque", "Rear torque", ".2f", "Nm", float),
            ("torque_gain", "Gain", ".1f", "", float),

            (None, None, None, None, None),
            ("drive_force", "Drive force", ".2f", "N", float),
            ("total_resistance", "Resistance", ".2f", "N", float),
            (None, None, None, None, None),
            ("rolling_resistance", "Rolling resistance", ".2f", "N", float),
            ("wind_drag", "Wind drag", ".2f", "N", float),
            # ("roll_gravity", "Wind drag", ".2f", "N", float),
            (None, None, None, None, None),
            ("front_gear", "Front", "", "", lambda x: "Outer" if x == 0 else "Inner"),
            ("rear_gear", "Rear", "", "", lambda x: ordinal(x + 1)),
            ("gear_ratio", "Current gear ratio", ".2f", "", float),
            ("steer", "Steer", ".1f", "deg", float),

        ]
        return self.stats_formatter(self.bike, keys)

    def rider_stats(self):
        keys = [
            ("power", "Power", ",", "W", int),
            ("cadence", "Cadence", ",", "rpm", int),
            ("required_power", "Power required", ",", "W", int),
            ("required_gear_ratio", "Required gear ratio", ".2f", "", float),
            (None, None, None, None, None),
            ("warning", "Status", "", "", str),
            ("fatigue", "Fatigue", "", "%", lambda v: int(v * 100)),
            (None, None, None, None, None),
            ("required_power", "Power required", ",", "W", int),
            ("rolling_resistance_power", "Rolling resistance", ",", "W", int),
            ("air_drag_power", "Air drag", ",", "W", int),
            ("slope_resistance_power", "Slope resistance", ",", "W", int),
            ("drive_loss_power", "Drivetrain loss", ",", "W", int),

        ]
        return self.stats_formatter(self.rider, keys)

    def stats_formatter(self, obj, keys):
        s = []
        for k, label, fmt, unit, func in keys:
            if k is None:
                s.append("")
                continue
            v = obj.get_status(k)
            v = func(v)
            f = "{:<20} {:>8" + fmt + "} {:>5}"
            s.append(f.format(label, v, unit))
        return '\n'.join(s)
