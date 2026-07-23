"""Scenario Description:

The scenario takes place on a long, straight road with at least two lanes divided by white dotted lines, where a blue Vehicle Under Test (VUT) is driving forward. Positioned in the same lane ahead of the VUT is a stationary shared bicycle, which is offset 0.5 meters to the right of the lane's center line. The entire test environment simulates a foggy day with visibility limited to between 150 and 200 meters.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

import carla

EGO_MODEL = 'vehicle.tesla.model3'
DISTANCE = 30

#################################
# WEATHER                       #
#################################

# Foggy weather with visibility limited to between 150 and 200 meters
param weather = Uniform(
    carla.WeatherParameters(sun_altitude_angle=30.0, fog_density=50.0, fog_distance=150.0),
    carla.WeatherParameters(sun_altitude_angle=30.0, fog_density=50.0, fog_distance=175.0),
    carla.WeatherParameters(sun_altitude_angle=30.0, fog_density=50.0, fog_distance=200.0)
)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car,
    with blueprint EGO_MODEL,
    with color Color.withBytes([0, 0, 255])

# Stationary shared bicycle ahead in the same lane, offset 0.5 m to the right
new Bicycle at ego offset by (DISTANCE, -0.5),
    with regionContainedIn ego.laneSection