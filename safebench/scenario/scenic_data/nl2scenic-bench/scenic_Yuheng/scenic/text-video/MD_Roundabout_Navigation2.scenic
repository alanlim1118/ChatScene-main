"""Scenario Description:

The ego vehicle travels straight on a multi-lane road towards an urban roundabout featuring a central island with a dark circular base and a white spiral structure. As the ego vehicle approaches the junction, a red adversary vehicle travels in the lane to its left, and both cars enter the roundabout at the same time. Meanwhile, other vehicles are visible navigating the circular intersection ahead, with residential buildings lining the left side of the road and a tree-lined pedestrian area on the right.

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
ADV_MODEL = "vehicle.tesla.model3"
BG_MODELS = ["vehicle.audi.a2", "vehicle.bmw.grandtourer", "vehicle.volkswagen.t2"]

param EGO_SPEED = Range(5, 8)
param ADV_SPEED = Range(5, 8)
param BG_SPEED = Range(4, 7)

param EGO_INIT_DIST = Range(30, 50)
param ADV_LATERAL_OFFSET = Range(3.5, 4.5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, 8):
        take SetBrakeAction(1)
        wait

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior BackgroundBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a roundabout intersection from the network
roundabout = Uniform(*filter(lambda i: i.isRoundabout, network.intersections))

# Ego enters the roundabout going straight through one of the incoming lanes
egoEntryManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, roundabout.incomingManeuvers))
egoInitLane = egoEntryManeuver.startLane
egoTrajectory = [egoInitLane, egoEntryManeuver.connectingLane] + list(roundabout.circulatingLanes)

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary is in the lane to the left of ego's lane
advLaneCandidates = filter(lambda l: l is not egoInitLane and 
                                    abs(l.centerline.start.heading - egoInitLane.centerline.start.heading) < 10 deg,
                           egoInitLane.leftLanes if hasattr(egoInitLane, 'leftLanes') else [])
advLane = Uniform(*advLaneCandidates) if advLaneCandidates else egoInitLane.adjacentLeftLane

advSpawnPt = new OrientedPoint in advLane.centerline,
    offset by (0, globalParameters.ADV_LATERAL_OFFSET) relative to egoSpawnPt

# Background vehicles already circulating in the roundabout
circulatingLane = Uniform(*roundabout.circulatingLanes)
bgSpawnPt1 = new OrientedPoint in circulatingLane.centerline
bgSpawnPt2 = new OrientedPoint in circulatingLane.centerline,
    following circulatingLane.orientation from bgSpawnPt1 for Range(15, 25)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(),
    with regionContainedIn None

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (0.8, 0.0, 0.0),
    with behavior AdversaryBehavior(),
    with regionContainedIn None

bgCar1 = new Car at bgSpawnPt1,
    with blueprint Uniform(*BG_MODELS),
    with behavior BackgroundBehavior(globalParameters.BG_SPEED),
    with regionContainedIn None

bgCar2 = new Car at bgSpawnPt2,
    with blueprint Uniform(*BG_MODELS),
    with behavior BackgroundBehavior(globalParameters.BG_SPEED),
    with regionContainedIn None

# Ensure ego starts at appropriate distance from roundabout entry
require EGO_INIT_DIST[0] <= (distance from ego to roundabout) <= EGO_INIT_DIST[1]

# Ensure adversary is roughly alongside ego so they enter simultaneously
require abs((distance from adversary to roundabout) - (distance from ego to roundabout)) <= 5

terminate when (distance from ego to roundabout) > 80 or simulation().currentTime > 30