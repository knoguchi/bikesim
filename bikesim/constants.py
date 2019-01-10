# -*- coding: utf-8 -*-
# Rolling resistance coeeficient
# P = Crr x N x v
# See https://en.wikipedia.org/wiki/Rolling_resistance
Crr = 0.0022  # Production bicycle tires at 120 psi (8.3 bar) and 50 km/h (31 mph), measured on rollers

# Air drag coefficient
# P = 0.5 x ρ x v^2 x Cd x A
# See https://en.wikipedia.org/wiki/Drag_coefficient
Cd = 0.88  # Road bicycle plus cyclist, touring position
# Rider
S = 0.36  # m^2 frontal surface

# Upright commuting bike 1.15 0.55
# Road bike, touring position 1.0 0.40
# Racing bike, rider crouched, tight clothing 0.88 0.36

# Drivetrain and bearing loss
# https://www.cyclingpowerlab.com/DrivetrainEfficiency.aspx
# https://ridefar.info/bike/cycling-speed/mechanical-resistance/
# Chain 3%, pedals 0.1%, BB 0.2%, derailleur 1%
Cb = 0.96

# Coefficients of Friction µ
u = 0.9  # Concrete or asphalt (dry) 0.8-0.9
# u = 0.7 # Concrete or asphalt (wet) 0.4-0.7
# u = 0.7 # Gravel 0.6-0.7
# u = 0.4 # Sand 0.3-0.4
# u = 0.2 # Icy 0.1-0.2

# Air density
Rho = 1.2041  # kg/m^3, at 20C, 101.325 kPa, dry air
