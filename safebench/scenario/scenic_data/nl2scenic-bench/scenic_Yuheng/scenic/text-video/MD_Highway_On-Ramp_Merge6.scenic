"""Scenario Description:

A top-down view captures a traffic scenario on a multi-lane highway bordered by residential buildings on the left and a forest on the right. A blue ego vehicle travels along an on-ramp curving from the bottom right, preparing to merge left into the highway's rightmost lane. Ahead of the ego vehicle on the ramp, a dark grey car completes its merge onto the main road. As the ego vehicle follows and merges into the same lane, a red adversary vehicle traveling on the highway is positioned behind the ego vehicle in the target lane. Further ahead in the left lane, a white car and another dark grey vehicle travel in the same direction, while the ego vehicle establishes its position in the right lane behind the initial merging vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_MERGE_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"
LEFT_LANE_MODEL_1 = "vehicle.nissan.micra"
LEFT_LANE_MODEL_2 = "vehicle.tesla.model3"

param EGO_SPEED = Range(8, 12)
param LEAD_MERGE_SPEED = Range(9, 13)
param ADV_SPEED = Range(10, 14)
param LEFT_CAR_1_SPEED = Range(9, 13)
param LEFT_CAR_2_SPEED = Range(8, 12)

param MERGE_DISTANCE = Range(30, 50)
param ADV_BEHIND_DIST = Range(15, 25)
param LEAD_AHEAD_DIST = Range(20, 35)
param LEFT_CAR_AHEAD_DIST = Range(40, 60)
param LEFT_CAR_SPACING = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoMergeBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, 5):
        take SetBrakeAction(1)

behavior LeadMergeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_MERGE_SPEED)

behavior AdvHighwayBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior LeftLaneBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable merge section: a lane that has a lane to its left (highway rightmost lane)
mergeTargetLanes = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToLeft is not None and sec._laneToLeft.isForward:
            mergeTargetLanes.append(sec)

targetLaneSec = Uniform(*mergeTargetLanes)
leftLaneSec = targetLaneSec._laneToLeft

# Spawn point for the lead merging car on the target lane
leadSpawnPt = new OrientedPoint in targetLaneSec.centerline

# Ego spawns behind the lead car on the same lane section (on-ramp portion)
egoSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for -globalParameters.LEAD_AHEAD_DIST

# Adversary spawns behind the ego in the target lane
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for -globalParameters.ADV_BEHIND_DIST

# Left lane cars spawn ahead in the left lane
leftCar1SpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.LEFT_CAR_AHEAD_DIST @ leftLaneSec
leftCar2SpawnPt = new OrientedPoint following roadDirection from leftCar1SpawnPt for globalParameters.LEFT_CAR_SPACING @ leftLaneSec

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "blue",
    with behavior EgoMergeBehavior()

leadMergeCar = new Car at leadSpawnPt,
    with heading leadSpawnPt.heading,
    with blueprint LEAD_MERGE_MODEL,
    with color "dark grey",
    with behavior LeadMergeBehavior()

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint ADV_MODEL,
    with color "red",
    with behavior AdvHighwayBehavior()

leftCar1 = new Car at leftCar1SpawnPt,
    with heading leftCar1SpawnPt.heading,
    with blueprint LEFT_LANE_MODEL_1,
    with color "white",
    with regionContainedIn leftLaneSec,
    with behavior LeftLaneBehavior(globalParameters.LEFT_CAR_1_SPEED)

leftCar2 = new Car at leftCar2SpawnPt,
    with heading leftCar2SpawnPt.heading,
    with blueprint LEFT_LANE_MODEL_2,
    with color "dark grey",
    with regionContainedIn leftLaneSec,
    with behavior LeftLaneBehavior(globalParameters.LEFT_CAR_2_SPEED)

require distance from ego to leadMergeCar >= 10
require distance from adversary to ego >= 8
require distance from leftCar1 to leftCar2 >= 10

terminate when (distance from ego to leadSpawnPt) > 150