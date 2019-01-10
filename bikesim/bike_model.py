# -*- coding: utf-8 -*-
from math import sin, cos, tan, radians, degrees, atan, sqrt, pi

from scipy.constants import g

from bikesim import Vector
from bikesim.constants import Crr, Cd, S, Rho


def km(v):
    return v * 3600 / 1000


class Bike:
    def __init__(self, wheelbase=1000, head_angle=90.0, weight=9.0):
        self._world_position = Vector(x=0, y=0, angle=0, lean_angle=0)
        self._steer_angle = 0.0
        self._v = 0.0
        self._wheelbase = wheelbase / 1000  # convert to meter
        self._bike_weight = weight
        self._m = 0  # total mass of bike and rider
        self._crank_r = 0.17
        self._tire_r = 0.700 / 2  # 700c tires
        self._status = {}

        # gears.  outer-top, inner-low
        self._front_chainrings = [52, 36]  # inner-outer 1-2
        self._rear_chainrings = [28, 25, 23, 21, 19, 17, 15, 14, 13, 12, 11]  # Low-Top 1-11

        self.set_steer(0)
        self.set_front_derailleur(0)
        self.set_rear_derailleur(0)
        self.set_head_angle(head_angle)

    def set_rider_weight(self, rider_weight):
        self._m = self._bike_weight + rider_weight

    def set_world_position(self, wp):
        self._world_position = wp

    def get_world_position(self):
        return self._world_position

    def set_head_angle(self, degree):
        self._lambda = radians(90 - degree)

    def set_front_derailleur(self, n):
        self._front_gear = n
        self._front_t = self._front_chainrings[n]
        self._front_chainring_r = self._front_t * 0.0127 / pi / 2
        self._status.update(front_gear=n)
        return self._front_t

    def set_rear_derailleur(self, n):
        self._rear_gear = n
        self._rear_t = self._rear_chainrings[n]
        self._rear_chainring_r = self._rear_t * 0.0127 / pi / 2
        self._status.update(rear_gear=n)
        return self._rear_t

    def shift_down_front(self):
        """
        Shift front to larger cog
        :return:
        """
        if self._front_gear > 0:
            self.set_front_derailleur(self._front_gear - 1)

    def shift_up_front(self):
        """
        Shift front to smaller cog
        :return:
        """
        if self._front_gear < len(self._front_chainrings) - 1:
            self.set_front_derailleur(self._front_gear + 1)

    def shift_down_rear(self):
        """
        Shift rear to larger cog
        :return:
        """
        if self._rear_gear < len(self._rear_chainrings) - 1:
            self.set_rear_derailleur(self._rear_gear + 1)

    def shift_up_rear(self):
        """
        Shift rear to smaller cog
        :return:
        """
        if self._rear_gear > 0:
            self.set_rear_derailleur(self._rear_gear - 1)

    def set_steer(self, degree):
        if -90 < degree < 90:
            self._steer_angle = radians(degree)
            self._status.update(steer=degree)
            return
        print("can't steer {} degree".format(degree))

    def steer_angle(self):
        return degrees(self._steer_angle)

    def speed(self, velocity_in_km=None):
        if velocity_in_km is not None:
            self._v = velocity_in_km * 1000 / 3600  # convert to v = meter/sec
        return km(self._v)

    def run(self, dt, input_power, cadence, u=0, bank_beta=0):
        """
        Calculate delta vector after dt second.
        This function does not update the world coordinate
        """
        self._status.update(v=self._v, cadence=cadence)

        # Driving force
        driving_force = self.f_drive_force(input_power, cadence)

        # Rolling resistance
        rr = self.f_rolling_resistance()

        # Air drag
        rd = self.f_wind()

        # slope
        if bank_beta > 0:
            phi = self.lean_angle()
            alpha = phi - bank_beta
        else:
            alpha = 0
        rs = self.f_roll_gravity(alpha)

        total_resistance = rr + rd + rs
        self._status.update(total_resistance=total_resistance)
        # HACK to supress vibration
        """
        if abs(self._v) < 1:
            total_resistance = 0
            self.speed(0)
        elif self._v < 0:
            # we don't care negative Fp aka fixie
            R = -R
        """

        accel = (driving_force - total_resistance) / self._m
        self._status.update(accelaration=accel)

        eps = 0.00001
        if -eps < self._steer_angle < eps:
            dx = self._v * dt
            dy = 0
            d_psi = 0
            lean_angle = 0
        else:

            # Calculate path for riding through curves
            delta = self._steer_angle
            zeta = delta * cos(self._lambda)

            # TODO: is it ok to allow negative r for turning right?
            r = self._wheelbase / tan(zeta)

            omega = self._v / r  # angular speed in rad
            d_psi = omega * dt  # actual angle traveled during time dt
            dx = r * sin(d_psi)
            dy = r * (1 - cos(d_psi))

            # calculate lean angle
            # Fc = mv^2 / r
            # Fg = mg
            # tan(phi) = mv^2 / mgr = v^2 / gr
            # phi = arctan(v^2 / gr)
            fc = self._m * (self._v ** 2.0) / r
            lean_angle = atan(self._v ** 2.0 / g / r)

            self._status.update(
                turning_r=r,
                centrifugal_force=fc / g,
                lean_angle=lean_angle
            )

            # TODO: counter steering. maybe it's already taken cared

            # Side slip
            # Fc = mv^2 / r
            # Fg = mg
            # FN = Fg
            # FT = μmg

            # Fc < FT (or slip)
            # mVmax^2 / r  < μmg
            # Vmax < sqrt(μrg)

            # dry asphalt 0.8
            # wet asphalt 0.4-0.6
            # snow 0.2-0.5
            # solid snow 0.2-0.35
            # ice 0.1-0.2

            # braking distance = v^2 / (2gμ)
            v_max = sqrt(abs(r) * g * u)
            self._status.update(v_max=v_max)

            # Banked Turn
            # alpha = ground normal vector - phi
            # beta = gravity normal vector
            # if phi == beta:
            #    # FR = FN
            # elif phi > beta: # case2
            #    alpha = beta + phi
            # else:  # case 3
            #    alpha = beta - phi
            # # Adhesive force FT
            # FT = uFN = uFR * cos(alpha)
            # FT = self._u * FR * cos(alpha)
            # FP = R*sin(alpha)
            # FT >= FP
            # FR sin(alpha) <= uFR cos(alpha)
            # tan alpha < u

            # FC/Fg = tan phi = tan (beta + alpha)  # case2
            #                 = tan (beta - alpha)  # case3
            # (mv^2/r) / mg = v^2 / rg = tan(beta + alpha)
            #                          = tan(beta - alpha)
            # v^2 = tan(beta + alpha) = rg (tan beta - tan alpha) / (1 + tan beta * tan alpha)
            # v^2 = tan(beta - alpha) = rg (tan beta + tan alpha) / (1 - tan beta * tan alpha)
            # because tan alpha <= u
            # vmax = sqrt(rg * ((tan beta + u)/(1 - u tan beta)))
            # vmin = sqrt(rg * ((tan beta - u)/(1 + u tan beta)))

        # finally update the velocity after dt of accelaration
        self._v += accel * dt
        return Vector(dx, dy, d_psi, lean_angle)

    def f_rolling_resistance(self):
        """
        Rolling Resistance Force
        """
        rr = self._m * g * Crr * self._v
        self._status.update(rolling_resistance=rr)
        return rr

    def f_wind(self):
        """
        Wind drag
        """
        wind_drag = 0.5 * Rho * (self._v ** 2) * Cd * S
        self._status.update(wind_drag=wind_drag)
        return wind_drag

    def f_roll_gravity(self, alpha):
        """
        Roll Gravity
        """
        roll_gravity = self._m * g * sin(alpha)
        self._status.update(roll_gravity=roll_gravity)
        return roll_gravity

    def f_drive_force(self, power, cadence):
        """
        calculate drive force
        pedal
        -> crank
        -> front_gear
        -> chain
        -> rear_gear
        -> rear wheel
        """
        if power > 0 and cadence > 0:
            # crank_torque = pedaling_f * self._crank_r * 9.8
            # power = crank_torque * radians(60 * 360 / rpm)
            crank_torque = power / radians(60 * 360 / cadence)
            chain_tension = crank_torque / self._front_chainring_r
            rear_hub_torque = chain_tension * self._rear_chainring_r
            drive_force = rear_hub_torque / self._tire_r
        else:
            crank_torque = 0
            chain_tension = 0
            rear_hub_torque = 0
            drive_force = 0

        self._status.update(
            power=power,
            drive_force=drive_force,
            crank_torque=crank_torque,
            chain_tension=chain_tension,
            rear_hub_torque=rear_hub_torque
        )
        return drive_force

    def brake(self, pct):
        """
        TODO: braking model.
        :return:
        """
        assert 0.0 < pct < 1.0
        return 0.56 * g * pct

    def get_status(self, k):
        return self._status.get(k)

    def __str__(self):
        s = []
        for k in sorted(self._status):
            v = self._status.get(k)
            s.append("{}: {}".format(k, v))
        return '\n'.join(s)
