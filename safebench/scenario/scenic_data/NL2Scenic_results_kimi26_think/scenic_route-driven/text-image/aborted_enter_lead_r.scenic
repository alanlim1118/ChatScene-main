"""Scenario Description:

The image displays a top-down view of a multi-lane road marked by dashed white lines, where a blue ego vehicle is positioned in the center lane with a straight blue arrow indicating forward motion. Ahead and to the right of the ego vehicle, a pink car is shown with a wavy pink arrow trajectory that initially curves upward toward the center lane before veering back down toward the right lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
middleLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = middleLaneSec._laneToRight

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in the center lane, facing forward
ego = new Car at egoSpawnPt,
    facing roadDirection,

# Pink car ahead in the right lane; project ego onto the right lane
# centerline and place the car further along the road
rightLaneStart = rightLaneSec.centerline.project(ego.position)
pinkSpawnPt = follow roadDirection from rightLaneStart for Range(20, 50)

# The pink car is angled slightly toward the center lane to reflect
# the initial part of its wavy trajectory
pinkCar = new Car at pinkSpawnPt,
    facing Range(5 deg, 15 deg) relative to roadDirection,
