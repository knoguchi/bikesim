from math import sin, cos, tan, atan, pi, sqrt, radians

from bikesim.constants import Rho, K, Cb, Cd, Standard_BMI, Standard_Height, G


class Rider:
    def __init__(self, weight=60.0):
        self._bike = None

        self.name = "base rider"
        self.height = 1.62
        self.weight = weight

        self.fatigue = 0.0

        self.stall_speed = 3.0  # TODO: consider bike types

        self.Pmin = 200.0
        self.Pmed = 400.0
        self.Pmax = 800.0

        # to calculate required power
        self.road_conditon = 1.0  # smooth 1, rough  1.2, no pavement 3.0
        self.trunk_angle = radians(43.5)
        self.clothes = 1.0  # bike wear 1, winter coat 1.15, summer 1.05, spring/autumn 1.1
        # surface
        self.face_w = 0.14
        self.shoulder_w = 0.45
        self.shoulder_t = 0.12
        self.trunk_w = 0.4
        self.arm2_w = 0.16
        self.leg2_w = 0.24
        self._status = {}

    def ride_on(self, bike):
        # TODO: cyclick ref is bad
        self.bike = bike
        self.bike.set_rider(self)

    def max_available_power(self):
        """
        Compute maximum available power by fatigue and cadence
        """
        if self.fatigue < 0.5:
            Pm = 2 * self.fatigue * (self.Pmed - self.Pmax) + self.Pmax
        else:
            Pm = 2 * (self.fatigue - 1 / 2) * (self.Pmin - self.Pmed) + self.Pmed
        self._status.update(max_power=Pm)
        return Pm

    def power_by_cadence(self, a_power, cadence):
        if 0 < cadence <= 85:
            c = (1 - cos(cadence * pi / 85)) * 0.25
        elif 85 < cadence < 170:
            c = (1 - cos(cadence * 2 * pi / 170)) * 0.5
        else:
            c = 0
        return a_power * c

    def update_fatigue(self, power_consumed, a_power):
        if power_consumed / a_power >= 0.50:
            # considered anaerobic
            self.fatigue += 0.1
            if self.fatigue > 1.0:
                self.fatigue = 1.0
        elif self.fatigue > 0:
            self.fatigue -= 0.1
            if self.fatigue < 0:
                self.fatigue = 0
        self._status.update(fatigue=self.fatigue)

    def body_surface_area(self):
        bmi = self.weight / pow(self.height, 2.0)
        body_index = sqrt(bmi / Standard_BMI)
        height_index = self.height / Standard_Height

        # black magic
        face_a = 0.12 * self.height * height_index * self.face_w
        shoulder_a = height_index * self.shoulder_w * height_index * self.shoulder_t
        trunk_a = 0.4 * self.height * height_index * self.trunk_w
        arm_a = 0.32 * self.height * height_index * self.arm2_w
        leg_a = 0.45 * self.height * height_index * self.leg2_w
        body_a = (face_a + trunk_a * sin(self.trunk_angle) + shoulder_a * cos(self.trunk_angle) +
                  arm_a * sin(self.trunk_angle) + leg_a)
        bsa = body_a * body_index * self.clothes  # bmi and clothes affect effective surface area
        return bsa

    def get_required_power(self, road_condition=1.0, tire_resist=0.0036, tire_width=0.028, clothes=1.0,
                           body_height=1.62, body_weight=65, bike_weight=10, cargo_weight=0,
                           tire_r=0.345, air_temp=20, head_wind=5, slope_grade_pct=10, trunk_angle_deg=43.5
                           ):

        face_w = 0.14
        shoulder_w = 0.45
        shoulder_t = 0.12
        trunk_w = 0.4
        arm2_w = 0.16
        leg2_w = 0.24

        # Road
        # road condition: smooth 1, rough  1.2, no pavement 3.0
        slope_grade = slope_grade_pct * 0.01
        trunk_angle = radians(trunk_angle_deg)
        velocity = self.bike.speed()  # km converted to m/s

        # ---------------- calculation -------------------------
        # Calculate surface area of the rider that takes air drag
        bmi = body_weight / pow(body_height, 2.0)
        body_index = sqrt(bmi / Standard_BMI)
        height_index = body_height / Standard_Height

        # black magic
        face_a = 0.12 * body_height * height_index * face_w
        shoulder_a = height_index * shoulder_w * height_index * shoulder_t
        trunk_a = 0.4 * body_height * height_index * trunk_w
        arm_a = 0.32 * body_height * height_index * arm2_w
        leg_a = 0.45 * body_height * height_index * leg2_w
        body_a = (face_a + trunk_a * sin(trunk_angle) + shoulder_a * cos(trunk_angle) +
                  arm_a * sin(trunk_angle) + leg_a)
        body_area = body_a * body_index * clothes

        # Calculate surface area of bike
        bike_area = (tire_r * 2) * tire_width

        # Total surface area
        total_area = body_area + bike_area

        # Relative velocity is ground speed + wind speed
        total_velo = velocity + head_wind

        # Total mass
        total_mass = body_weight + bike_weight + cargo_weight

        # --------------- calculate resistance --------------------------
        # Rolling Resistance
        rolling_resistance = road_condition * tire_resist * total_mass * G

        # Air Drag
        density = Rho * K / (K + air_temp)
        air_drag = Cd * density * total_area * pow(total_velo, 2) / 2

        # Gravity at slope
        slope_angle = atan(slope_grade)
        slope_resistance = sin(slope_angle) * total_mass * G

        # Total Resistance
        total_resistance = rolling_resistance + air_drag + slope_resistance

        # ---------------- calculate power loss --------------------------
        rolling_resistance_power = rolling_resistance * velocity
        air_drag_power = air_drag * velocity
        slope_resistance_power = slope_resistance * velocity
        drive_loss_power = (1 - Cb) * total_resistance * velocity

        # Required power = Total Power loss
        total_required_power = rolling_resistance_power + air_drag_power + slope_resistance_power + drive_loss_power

        return total_required_power

    def get_required_power2(self, cargo_weight=0, air_temp=20, head_wind=0, slope_grade_pct=0, speed_km=5):

        # Road
        # road condition: smooth 1, rough  1.2, no pavement 3.0
        slope_grade = slope_grade_pct * 0.01
        velocity = self.bike.speed()  # km converted to m/s

        # Relative velocity is ground speed + wind speed
        total_velo = self.bike.speed() + head_wind

        # --------------- calculate resistance --------------------------
        # Rolling Resistance
        rolling_resistance = self.bike.f_rolling_resistance()

        # Air Drag
        air_drag = self.bike.f_wind(v=total_velo, air_temp=air_temp)

        # Gravity at slope
        slope_angle = atan(slope_grade)
        slope_resistance = sin(slope_angle) * (self.bike.total_weight + cargo_weight) * G

        # Total Resistance
        total_resistance = rolling_resistance + air_drag + slope_resistance

        # ---------------- calculate power loss --------------------------
        rolling_resistance_power = rolling_resistance * velocity
        air_drag_power = air_drag * velocity
        slope_resistance_power = slope_resistance * velocity
        drive_loss_power = (1 - Cb) * total_resistance * velocity

        self._status.update(
            rolling_resistance_power=rolling_resistance_power,
            air_drag_power=air_drag_power,
            slope_resistance_power=slope_resistance_power,
            drive_loss_power=drive_loss_power
        )
        # Required power = Total Power loss
        total_required_power = rolling_resistance_power + air_drag_power + slope_resistance_power + drive_loss_power

        return total_required_power

    def required_gear_ratio(self, power, cadence, slope_grade_pct):
        """
        Wattage must be sustainable (e.g. 1hr)
        cadence > 60rpm is said to be easier
        :param form:
        :return:
        """
        if power > 0 and cadence > 0:
            prop = self.bike.bike_prop('road')  # roadbike 0.0036, mtb 0.012, cross 0.004
            tire_resistance = prop.get('resist')

            alpha = atan(0.01 * slope_grade_pct)
            slope_resistance = tan(alpha)
            gear_ratio = round(
                600 * power / (
                        (
                                tire_resistance + slope_resistance
                        ) *
                        self.bike.total_weight *
                        G *
                        pi *
                        self.bike._tire_r * 2 *
                        cadence
                )
            ) / 10
            return gear_ratio
        return 10

    def run(self, dt, cadence, u):
        """
        :param freq: pedaling frequency in rpm
        :return: power in wattage
        """
        # print("XXXX")
        # p = self.get_required_power()
        # print(p)
        #
        required_power = self.get_required_power2()
        # print(p2)
        # print("YYY")
        # TODO: control fatigue by timer using dt
        a_power = self.max_available_power()
        power_applied = self.power_by_cadence(a_power, cadence)
        self.update_fatigue(power_applied, a_power)
        req_gear_ratio = self.required_gear_ratio(power_applied, cadence, 0)
        req_gear_ratio = power_applied / (required_power + 0.1)
        self._status.update(power=power_applied,
                            cadence=cadence,
                            required_gear_ratio=req_gear_ratio,
                            required_power=required_power,
                            )
        if required_power < power_applied:
            if self.bike.shift_down_rear():
                pass
            elif self.bike.shift_down_front():
                pass
            else:
                self._status.update(warning="Too steep!!")
        else:
            if self.bike._rear_gear > 0:
                self.bike.shift_up_rear()
            elif self.bike._front_gear > 0:
                self.bike.shift_up_front()
            else:
                self._status.update(warning="Pretty good!!")

        vector = self.bike.run(dt, power_applied, cadence, u)
        return vector

    def get_status(self, k):
        return self._status.get(k)

    def __str__(self):
        s = []
        for k in sorted(self._status):
            v = self._status.get(k)
            s.append("{}: {}".format(k, v))
        return '\n'.join(s)
