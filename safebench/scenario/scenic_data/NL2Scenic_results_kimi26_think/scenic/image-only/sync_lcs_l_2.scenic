"""Scenario Description:

The blue ego vehicle is traveling in the right lane and executing a lane change maneuver to the left lane. Ahead of the ego vehicle in the right lane, a leading pink car is simultaneously merging into the same left target lane. At the same time, a trailing pink car is traveling in the left target lane, approaching from behind the ego vehicle's current position, requiring the ego vehicle to coordinate its merge safely between the leading merging vehicle and the trailing vehicle in the target lane.

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

param OPT_EGO_SPEED = Range(6, 8)
param OPT_LEADING_SPEED = Range(5, 7)
param OPT_TRAILING_SPEED = Range(9, 11)

param OPT_LEADING_DISTANCE = Range(25, 35)
param OPT_TRAILING_LONG_OFFSET = Range(-30, -20)
param OPT_TRAILING_LAT_OFFSET = Range(-4.0, -3.5)

param OPT_EGO_MERGE_TRIGGER = Range(5, 10)
param OPT_LEADING_MERGE_TRIGGER = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to egoSpawnPt) > globalParameters.OPT_EGO_MERGE_TRIGGER
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior LeadingPinkBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED) until (distance from self to leadingSpawnPt) > globalParameters.OPT_LEADING_MERGE_TRIGGER
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.OPT_LEADING_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

behavior TrailingPinkBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRAILING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            laneSec._laneToLeft.isForward
        ):
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DISTANCE

trailingShift = globalParameters.OPT_TRAILING_LAT_OFFSET @ globalParameters.OPT_TRAILING_LONG_OFFSET
trailingSpawnPt = new OrientedPoint at egoSpawnPt offset along egoSpawnPt.heading by trailingShift

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color [0.1, 0.1, 0.8],
    with behavior EgoBehavior()

LeadingPink = new Car at leadingSpawnPt,
    with heading leadingSpawnPt.heading,
    with regionContainedIn None,
    with color [0.9, 0.4, 0.6],
    with behavior LeadingPinkBehavior()

TrailingPink = new Car at trailingSpawnPt,
    with heading trailingSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with color [0.9, 0.4, 0.6],
    with behavior TrailingPinkBehavior()

require (distance from LeadingPink to ego) > 20
require (distance from TrailingPink to ego) > 15
require distance to intersection >= 100
terminate when (distance from ego to egoSpawnPt) > 80