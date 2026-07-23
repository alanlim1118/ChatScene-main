"""Scenario Description:

The scenario takes place on a long, straight road with at least two lanes divided by white dotted lines, where a blue Vehicle Under Test (VUT) is driving forward. Positioned in the same lane ahead of the VUT is a stationary shared bicycle, which is offset 0.5 meters to the right of the lane's center line. The entire test environment simulates a foggy day with visibility limited to between 150 and 200 meters.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

EGO_MODEL = 'vehicle.tesla.model3'
BICYCLE_MODEL = 'vehicle.bh.crossbike'

# Fog visibility between 150 and 200 meters
fog_distance = Uniform(150, 200)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue VUT driving forward on a straight road
ego = new Car,
    with blueprint EGO_MODEL,
    with color Color.withBytes([0, 0, 255]),
    with velocity Range(8, 12) along roadDirection

# Stationary shared bicycle in the same lane, 30m ahead, offset 0.5m right of lane center
bicycle = new Bicycle following roadDirection from ego for 30,
    with blueprint BICYCLE_MODEL,
    with regionContainedIn ego.laneSection,
    with velocity 0,
    at Offset(0.5, 0) relative to ego.laneSection.centerLine

# Apply fog weather parameters
param fogDistance = fog_distance

#################################
# REQUIREMENTS                  #
#################################

require ego.laneSection.leftLane is not None or ego.laneSection.rightLane is not None