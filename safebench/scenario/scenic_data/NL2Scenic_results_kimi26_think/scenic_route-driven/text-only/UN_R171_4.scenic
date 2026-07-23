"""Scenario Description:

While traveling on a straight section, the ego vehicle wants to perform a lane change to overtake a lead vehicle but remains in its current lane because it detects a motorcycle filtering or approaching rapidly from behind in the adjacent lane, ensuring the target space is clear before moving.

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

LEAD_SPEED = 6
MOTORCYCLE_SPEED = 18
EGO_TO_LEAD = 20
MOTORCYCLE_START = 25

#################################
# AGENT BEHAVIORS               #
#################################

# LEAD VEHICLE BEHAVIOR: Slow moving
behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

# MOTORCYCLE BEHAVIOR: Fast moving in adjacent lane
behavior MotorcycleBehavior():
    do FollowLaneBehavior(target_speed=MOTORCYCLE_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
targetLaneSec = egoLaneSec._laneToLeft

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for EGO_TO_LEAD

# Motorcycle spawn point in target lane, behind ego
targetLanePt = targetLaneSec.centerline.project(egoSpawnPt.position)
motorcycleSpawnPt = new OrientedPoint following roadDirection from targetLanePt for -MOTORCYCLE_START

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec

leadVehicle = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior LeadBehavior()

motorcycle = new Car at motorcycleSpawnPt,
    with regionContainedIn targetLaneSec,
    with blueprint "vehicle.yamaha.yzf",
    with behavior MotorcycleBehavior()

# Ensure straight section away from intersections
require (distance from ego to intersection) > 50
require (distance from leadVehicle to intersection) > 50
require (distance from motorcycle to intersection) > 50

terminate when distance from ego to egoSpawnPt > 150
