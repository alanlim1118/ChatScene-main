"""Scenario Description:

In this top-down aerial view of a residential T-intersection, a green ego vehicle travels eastbound on the horizontal main road and initiates a right turn into the vertical side street, following a curved green trajectory line. Simultaneously, a blue adversarial vehicle approaches from the west on the opposite side of the main road and executes a left turn into the same vertical street, marked by a yellow trajectory line, creating a conflict as both vehicles attempt to merge into the same lane. A red car follows directly behind the green ego vehicle, while a yellow car trails the blue adversarial vehicle. The scene is framed by a row of townhouses at the top, a large modern building on the bottom left, and a grassy field on the bottom right, with trees lining the sidewalks and a "STOP" marking painted on the road surface at the bottom entrance of the vertical street.

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
ADV_MODEL = "vehicle.tesla.model3"
FOLLOWER_EGO_MODEL = "vehicle.audi.tt"
FOLLOWER_ADV_MODEL = "vehicle.mustang"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(3, 6)
param OPT_FOLLOWER_DISTANCE = Range(8, 15)
param OPT_BRAKE_DIST = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior AdvBehavior():
    try:
        do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior FollowerBehavior(leader, follow_distance):
    try:
        do FollowLeaderBehavior(leader, target_speed=globalParameters.OPT_EGO_SPEED, distance=follow_distance)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-intersection where ego turns right and adv turns left into same lane
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego: eastbound right turn
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: westbound left turn into same end lane as ego
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.endLane is egoManeuver.endLane, intersection.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Follower positions: behind their respective leaders along the start lanes
followerEgoSpawnPt = new OrientedPoint behind egoSpawnPt by globalParameters.OPT_FOLLOWER_DISTANCE
followerAdvSpawnPt = new OrientedPoint behind advSpawnPt by globalParameters.OPT_FOLLOWER_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Green ego vehicle making right turn
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0, 255, 0),
    with behavior EgoBehavior()

# Blue adversarial vehicle making conflicting left turn
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0, 0, 255),
    with behavior AdvBehavior()

# Red follower behind ego
FollowerEgo = new Car at followerEgoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint FOLLOWER_EGO_MODEL,
    with color (255, 0, 0),
    with behavior FollowerBehavior(ego, globalParameters.OPT_FOLLOWER_DISTANCE)

# Yellow follower behind adversarial vehicle
FollowerAdv = new Car at followerAdvSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint FOLLOWER_ADV_MODEL,
    with color (255, 255, 0),
    with behavior FollowerBehavior(AdvAgent, globalParameters.OPT_FOLLOWER_DISTANCE)

# Ensure proper spacing and conflict geometry
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from advSpawnPt to intersection) <= 40
require abs((egoSpawnPt.heading - advSpawnPt.heading) % 360 deg - 180 deg) < 30 deg