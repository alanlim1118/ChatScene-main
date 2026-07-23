"""Scenario Description:

The ego-vehicle encounters an obstacle blocking the lane and must perform a lane change into traffic moving in the opposite direction to avoid it. The obstacle may be a construction site, an accident or a parked vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
OBSTACLE_MODEL = Uniform(
    "static.prop.constructioncone",
    "vehicle.carlamotors.firetruck",
    "vehicle.nissan.patrol"
)

param EGO_SPEED = Range(4, 7)
param OBSTACLE_DIST = Range(30, 50)
param ONCOMING_DIST = Range(60, 90)
param ONCOMING_SPEED = Range(5, 9)
param LANE_CHANGE_TRIGGER_DIST = 18
param SAFE_RETURN_DIST = 10
param BRAKE_DIST = 12
param TERM_DIST = 120

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) \
            until (distance from self to Obstacle) < globalParameters.LANE_CHANGE_TRIGGER_DIST
        # Change into opposite lane to bypass obstacle
        oppLaneSec = egoLaneSec._laneToLeft
        do LaneChangeBehavior(
                laneSectionToSwitch=oppLaneSec,
                is_oppositeTraffic=True,
                target_speed=globalParameters.EGO_SPEED)
        # Drive in opposite lane until past obstacle
        do FollowLaneBehavior(
                target_speed=globalParameters.EGO_SPEED,
                laneToFollow=oppLaneSec.lane) \
            until (distance from self to Obstacle) > globalParameters.SAFE_RETURN_DIST
        # Return to original lane
        do LaneChangeBehavior(
                laneSectionToSwitch=egoLaneSec,
                is_oppositeTraffic=False,
                target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance from self to OncomingCar) < globalParameters.BRAKE_DIST \
                   and self.laneSection is not egoLaneSec:
        # Emergency brake if oncoming car is too close while in opposite lane
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)

behavior OncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an opposing left lane for borrowing
validLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None:
            leftLane = laneSec._laneToLeft
            if leftLane.isForward != laneSec.isForward:
                validLaneSecs.append(laneSec)

require len(validLaneSecs) > 0

egoLaneSec = Uniform(*validLaneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place obstacle ahead in ego's lane
obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt \
    for globalParameters.OBSTACLE_DIST

# Place oncoming car in the opposite lane, approaching toward ego
oppLaneSec = egoLaneSec._laneToLeft
oncomingBasePt = new OrientedPoint following roadDirection from obstacleSpawnPt \
    for globalParameters.ONCOMING_DIST
oncomingSpawnPt = oppLaneSec.centerline.project(oncomingBasePt.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

Obstacle = new Car at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint OBSTACLE_MODEL

OncomingCar = new Car at oncomingSpawnPt,
    with heading oncomingSpawnPt.heading + 180 deg,
    with regionContainedIn oppLaneSec,
    with behavior OncomingBehavior()

require distance to intersection > 80
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST