"""Scenario Description:

The scenario depicts a three-lane roadway where the ego vehicle (blue) is positioned in the center lane and executes a lane change maneuver to the right lane. Simultaneously, a second vehicle (pink) is located in the left lane, traveling straight forward, and is positioned slightly behind the ego vehicle. This represents a "Lane change right with following object" scenario.

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
PINK_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_PINK_SPEED = Range(8, 12)
param OPT_PINK_BEHIND = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed, target_lane):
    do FollowLaneBehavior(target_speed=target_speed) for Range(1, 3) seconds
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftAndRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftAndRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRightLane)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

# Pink vehicle spawn point in left lane, slightly behind ego
leftLaneProj = leftLaneSec.centerline.project(egoSpawnPt.position)
pinkSpawnPt = new OrientedPoint following roadDirection from leftLaneProj for -globalParameters.OPT_PINK_BEHIND

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (blue) in center lane, changing right
ego = new Car at egoSpawnPt,
    facing roadDirection,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, rightLaneSec)

# Pink vehicle in left lane, slightly behind, going straight
FollowingAgent = new Car at pinkSpawnPt,
    facing roadDirection,
    with regionContainedIn leftLaneSec,
    with blueprint PINK_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_PINK_SPEED)

require distance to intersection >= 50