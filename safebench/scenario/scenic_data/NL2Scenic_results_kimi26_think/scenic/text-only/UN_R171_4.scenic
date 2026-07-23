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

EGO_SPEED = 10
LEAD_SPEED = 6
MOTORCYCLE_SPEED = 18
EGO_TO_LEAD = 20
MOTORCYCLE_START = 25
OVERTAKE_DIST = 12
MOTORCYCLE_DIST = 10

#################################
# AGENT BEHAVIORS               #
#################################

# EGO BEHAVIOR: Follow lane, attempt overtake, but abort if motorcycle is too close in target lane
behavior EgoBehavior(target_lane_sec):
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED) until (distance from self to leadVehicle < OVERTAKE_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=EGO_SPEED)
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when distance from self to motorcycle < MOTORCYCLE_DIST:
        do FollowLaneBehavior(target_speed=EGO_SPEED)

# LEAD VEHICLE BEHAVIOR: Slow moving
behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

# MOTORCYCLE BEHAVIOR: Fast moving in adjacent lane
behavior MotorcycleBehavior():
    do FollowLaneBehavior(target_speed=MOTORCYCLE_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
    'No lane sections with adjacent left lane in network.'

egoLaneSec = Uniform(*laneSecsWithLeftLane)
targetLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for EGO_TO_LEAD

# Motorcycle spawn point in target lane, behind ego
targetLanePt = targetLaneSec.centerline.project(egoSpawnPt.position)
motorcycleSpawnPt = new OrientedPoint following roadDirection from targetLanePt for -MOTORCYCLE_START

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior EgoBehavior(targetLaneSec)

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