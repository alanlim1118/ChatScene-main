"""Scenario Description:

The ego-vehicle encounters an obstacle blocking the lane and must perform a lane change into traffic moving in the opposite direction to avoid it. The obstacle may be a construction site, an accident or a parked vehicle.

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
OBSTACLE_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_OBSTACLE_DIST = Range(30, 50)
param OPT_ADV_START_DIST = Range(50, 80)
param OPT_EGO_BYPASS_DISTANCE = Range(15, 25)
param OPT_EGO_MIN_BYPASS_DISTANCE = 10
param OPT_AVOIDANCE_DIST = Range(10, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to Obstacle < globalParameters.OPT_EGO_BYPASS_DISTANCE)
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to Obstacle > globalParameters.OPT_EGO_MIN_BYPASS_DISTANCE)
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_AVOIDANCE_DIST) and (AdvAgent.laneSection == self.laneSection):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

ObstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_OBSTACLE_DIST

adjLaneSec = egoLaneSec._laneToLeft

AdvSpawnPt = new OrientedPoint following roadDirection from ObstacleSpawnPt for globalParameters.OPT_ADV_START_DIST
AdvSpawnPt = adjLaneSec.centerline.project(AdvSpawnPt.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Obstacle setup (parked vehicle blocking the lane)
Obstacle = new Car at ObstacleSpawnPt,
    with heading ObstacleSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint OBSTACLE_MODEL

# Adversary car setup: oncoming traffic in the opposite lane
AdvAgent = new Car at AdvSpawnPt,
    with heading ObstacleSpawnPt.heading + 180 deg,
    with regionContainedIn adjLaneSec,
    with behavior AdvBehavior()

require distance to intersection > 80
terminate when distance from ego to egoSpawnPt > 120