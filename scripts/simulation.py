from bikesim.bike_model import Bike
from bikesim.rider_model import Rider
from bikesim.simulator import Simulator

mybike = Bike(wheelbase=975, head_angle=71.5, weight=10)
rider = Rider(weight=65)

sim = Simulator(mybike, rider, dt=0.1)
sim.run()
