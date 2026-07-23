"""Scenario Description:

Vehicle is backing up in an urban area, in daylight, under clear weather conditions, at a driveway or alley location, with a posted speed limit of 25 mph; and collides with another vehicle

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

WEATHER_OPTIONS = ['ClearNoon', 'ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# SPATIAL RELATIONS             #
#################################

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle backing up (facing opposite to lane direction)
ego = new Car on select_lane,
    facing (roadDirection at ego) + 180 deg

# Another vehicle in the backing path, resulting in collision
new Car at ego offset by 3 @ 0,
    facing roadDirection,
    with regionContainedIn ego.laneSection