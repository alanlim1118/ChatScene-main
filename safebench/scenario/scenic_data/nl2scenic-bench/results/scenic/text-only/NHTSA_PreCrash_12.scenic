"""Scenario Description:

Vehicle is backing up in an urban area, in daylight, under clear weather conditions, at a driveway or alley location, with a posted speed limit of 25 mph; and collides with another vehicle.

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

WEATHER_OPTIONS = ['ClearNoon', 'ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

SPEED_LIMIT_MPH = 25
SPEED_LIMIT_KMH = SPEED_LIMIT_MPH * 1.60934

#################################
# SCENARIO SPECIFICATION        #
#################################

# Select a road segment that could represent a driveway/alley (narrower roads in urban town)
candidate_roads = [r for r in network.roads if r.speedLimit <= SPEED_LIMIT_KMH + 5]
select_road = Uniform(*candidate_roads) if candidate_roads else Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)

# Ego vehicle positioned on the selected lane, facing backward to simulate backing up
ego = new Car on select_lane.centerline,
    with heading opposite roadDirection at self.position,
    with speed Range(-3, -1)  # negative speed indicates reversing

# Ensure ego is in a region consistent with driveway/alley context
require ego.laneSection is not None

# Place another vehicle behind the ego (in the direction ego is reversing toward)
# so that a collision occurs while backing up
other_car = new Car following roadDirection from ego for Range(5, 15),
    with regionContainedIn ego.laneSection,
    with speed 0

# Require that the two vehicles are close enough to collide during simulation
require distance between ego and other_car < 20

# Record collision as a requirement for scenario validity
record collision between ego and other_car