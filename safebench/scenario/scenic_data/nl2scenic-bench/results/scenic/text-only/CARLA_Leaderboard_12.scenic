"""Scenario Description:

The ego-vehicle encounters an obstacle blocking the lane and must perform a lane change into traffic moving in the same direction to avoid it. The obstacle may be a construction site, an accident or a parked vehicle.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
OBSTACLE_MODELS = ["static.prop.constructioncone", "static.prop.container", "vehicle.nissan.patrol"]

param EGO_SPEED = Range(8, 12)
param OBSTACLE_DIST = Range(40, 60)
param TRAFFIC_SPEED = Range(8, 12)
param TRAFFIC_GAP_AHEAD = Range(15, 25)
param TRAFFIC_GAP_BEHIND = Range(15, 25)
param LANE_CHANGE_TRIGGER_DIST = Range(18, 25)
param SAFE_RETURN_DIST = Range(10, 15)
param TERM_DIST = 120

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) \
            until (distance from self to obstacle) < globalParameters.LANE_CHANGE_TRIGGER_DIST
        do LaneChangeBehavior(
                laneSectionToSwitch=targetLaneSec,
                target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(
                target_speed=globalParameters.EGO_SPEED,
                laneToFollow=targetLaneSec.lane) \
            until (distance from self to obstacle) > globalParameters.SAFE_RETURN_DIST
        do LaneChangeBehavior(
                laneSectionToSwitch=egoLaneSec,
                target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1)

behavior TrafficBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.TRAFFIC_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a same-direction adjacent lane for overtaking
validLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            rightLane = laneSec._laneToRight
            leftLane = laneSec._laneToLeft
            if rightLane is not None and rightLane.isForward:
                validLaneSecs.append((laneSec, rightLane))
            elif leftLane is not None and leftLane.isForward:
                validLaneSecs.append((laneSec, leftLane))

require len(validLaneSecs) > 0
chosenPair = Uniform(*validLaneSecs)
egoLaneSec = chosenPair[0]
targetLaneSec = chosenPair[1]

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt \
    for globalParameters.OBSTACLE_DIST

# Project spawn points onto target lane for traffic vehicles
targetProjBase = targetLaneSec.centerline.project(obstacleSpawnPt.position)
trafficAheadSpawnPt = new OrientedPoint following roadDirection from targetProjBase \
    for globalParameters.TRAFFIC_GAP_AHEAD
trafficBehindSpawnPt = new OrientedPoint following roadDirection from targetProjBase \
    for -globalParameters.TRAFFIC_GAP_BEHIND

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

obstacle = new Object at obstacleSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint Uniform(*OBSTACLE_MODELS)

trafficAhead = new Car at trafficAheadSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint EGO_MODEL,
    with behavior TrafficBehavior()

trafficBehind = new Car at trafficBehindSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint EGO_MODEL,
    with behavior TrafficBehavior()

require (distance to intersection) > 80
require always (targetLaneSec is not None)
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST