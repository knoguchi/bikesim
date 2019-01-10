from math import cos, pi


class Rider:
    def __init__(self, weight=60.0):
        self._bike = None

        self.name = "base rider"
        self.weight = weight
        self.fatigue = 0.0

        self.Pmin = 200.0
        self.Pmed = 400.0
        self.Pmax = 800.0

        self._status = {}

    def ride_on(self, bike):
        self.bike = bike
        self.bike.set_rider_weight(self.weight)

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

    def run(self, dt, cadence, u):
        """
        :param freq: pedaling frequency in rpm
        :return: power in wattage
        """
        # TODO: control fatigue by timer using dt
        a_power = self.max_available_power()
        power_applied = self.power_by_cadence(a_power, cadence)
        self.update_fatigue(power_applied, a_power)
        self._status.update(power=power_applied, cadence=cadence)

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
