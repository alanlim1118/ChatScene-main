"""Scenario Description:

A top-down view captures a traffic scenario on a multi-lane highway bordered by residential buildings on the left and a forest on the right. A blue ego vehicle travels along an on-ramp curving from the bottom right, preparing to merge left into the highway's rightmost lane. Ahead of the ego vehicle on the ramp, a dark grey car completes its merge onto the main road. As the ego vehicle follows and merges into the same lane, a red adversary vehicle traveling on the highway is positioned behind the ego vehicle in the target lane. Further ahead in the left lane, a white car and another dark grey vehicle travel in the same direction, while the ego vehicle establishes its position in the right lane behind the initial merging vehicle.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_MERGE_SPEED = Range(10, 14)
param OPT_ADV_SPEED = Range(10, 14)
param OPT_LANE_SPEED = Range(12, 16)

param OPT_EGO_MERGE_DIST = Range(15, 25)
param OPT_ADV_TO_REF_DIST = Range(15, 25)
param OPT_MERGE_TO_REF_DIST = Range(20, 30)
param OPT_LEFT_AHEAD_DIST = Range(40, 60)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_lane_section):
    # Follow the on-ramp and merge left into the highway rightmost lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to mergingCar < globalParameters.OPT_EGO_MERGE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_section, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior HighwayFollowBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify a rightmost forward lane with a forward lane to its left.
# This serves as the on-ramp / merging lane.
rampLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            laneSec._laneToLeft.isForward and
            laneSec._laneToRight is None
        ):
            rampLaneSecs.append(laneSec)

rampLaneSec = Uniform(*rampLaneSecs)
targetLaneSec = rampLaneSec._laneToLeft
leftLaneSec = targetLaneSec._laneToLeft

# Reference point on the target highway lane
refPt = new OrientedPoint in targetLaneSec.centerline

# Dark grey merging car: ahead in the target lane, having completed its merge
mergeSpawnPt = new OrientedPoint following roadDirection from refPt for globalParameters.OPT_MERGE_TO_REF_DIST

# Red adversary: behind the ego in the target lane
advSpawnPt = new OrientedPoint following roadDirection from refPt for -globalParameters.OPT_ADV_TO_REF_DIST

# Ego on the on-ramp, positioned roughly opposite the reference point
egoSpawnPt = new OrientedPoint in rampLaneSec.centerline
require (distance from egoSpawnPt to refPt) < 6

# Left lane vehicles: further ahead
leftSpawnPt1 = new OrientedPoint in leftLaneSec.centerline
leftSpawnPt2 = new OrientedPoint in leftLaneSec.centerline
require (distance from leftSpawnPt1 to refPt) > globalParameters.OPT_LEFT_AHEAD_DIST
require (distance from leftSpawnPt2 to refPt) > (globalParameters.OPT_LEFT_AHEAD_DIST + 15)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue ego vehicle on the on-ramp preparing to merge left
ego = new Car at egoSpawnPt,
    with color (0.1, 0.3, 1.0),
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetLaneSec)

# Dark grey car ahead that has completed its merge onto the main road
mergingCar = new Car at mergeSpawnPt,
    with color (0.2, 0.2, 0.2),
    with heading mergeSpawnPt.heading,
    with regionContainedIn None,
    with behavior HighwayFollowBehavior(globalParameters.OPT_MERGE_SPEED)

# Red adversary vehicle in the target lane behind the ego
adversary = new Car at advSpawnPt,
    with color (0.9, 0.1, 0.1),
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior HighwayFollowBehavior(globalParameters.OPT_ADV_SPEED)

# White car in the left lane ahead
whiteCar = new Car at leftSpawnPt1,
    with color (1.0, 1.0, 1.0),
    with heading leftSpawnPt1.heading,
    with regionContainedIn None,
    with behavior HighwayFollowBehavior(globalParameters.OPT_LANE_SPEED)

# Another dark grey vehicle in the left lane ahead
darkGreyCar = new Car at leftSpawnPt2,
    with color (0.25, 0.25, 0.25),
    with heading leftSpawnPt2.heading,
    with regionContainedIn None,
    with behavior HighwayFollowBehavior(globalParameters.OPT_LANE_SPEED)

require leftLaneSec is not None
require distance to intersection >= 100
terminate when (distance from ego to mergingCar) > 80