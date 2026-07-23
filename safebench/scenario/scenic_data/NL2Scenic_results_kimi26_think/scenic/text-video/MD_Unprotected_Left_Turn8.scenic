"""Scenario Description:

Ego vehicle approaches a four-way urban intersection on a multi-lane road under clear weather conditions and prepares to turn left. Upon entering the intersection, the ego vehicle yields to an oncoming adversary vehicle traveling straight from the opposite direction, as well as to three adversary vehicles crossing from the right arm. As the vehicular traffic clears, a pedestrian is seen navigating through the intersection, crossing from the ego vehicle's right side toward the left, allowing the ego vehicle to complete its left turn and proceed along the cross street.

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

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior PedestrianBehavior():
    while True:
        take WalkForwardAction(1.4)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego setup: left turn
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Oncoming adversary: straight from opposite direction
egoStraight = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
oncomingStraight = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoStraight.reverseManeuvers))
oncomingLane = oncomingStraight.startLane
oncomingTrajectory = [oncomingLane, oncomingStraight.connectingLane, oncomingStraight.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingLane.centerline

# Right arm adversaries: three vehicles crossing from the right
conflictingStraight = list(filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
rightArmManeuvers = [m for m in conflictingStraight if m.startLane != oncomingLane]
rightArmLanes = [m.startLane for m in rightArmManeuvers]

advLane1 = Uniform(*rightArmLanes)
advLane2 = Uniform(*rightArmLanes)
advLane3 = Uniform(*rightArmLanes)

advManeuver1 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advLane1.maneuvers))
advManeuver2 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advLane2.maneuvers))
advManeuver3 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advLane3.maneuvers))

advTrajectory1 = [advLane1, advManeuver1.connectingLane, advManeuver1.endLane]
advTrajectory2 = [advLane2, advManeuver2.connectingLane, advManeuver2.endLane]
advTrajectory3 = [advLane3, advManeuver3.connectingLane, advManeuver3.endLane]

advSpawnPt1 = new OrientedPoint in advLane1.centerline
advSpawnPt2 = new OrientedPoint in advLane2.centerline
advSpawnPt3 = new OrientedPoint in advLane3.centerline

# Pedestrian crossing from right to left relative to ego
pedStart = new OrientedPoint at egoSpawnPt offset by (22, 3)
pedEnd = new OrientedPoint at egoSpawnPt offset by (22, -3)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

oncomingAdv = new Car at oncomingSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=oncomingTrajectory)

adv1 = new Car at advSpawnPt1,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory1)

adv2 = new Car at advSpawnPt2,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory2)

adv3 = new Car at advSpawnPt3,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory3)

ped = new Pedestrian at pedStart,
    facing toward pedEnd,
    with behavior PedestrianBehavior()

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from oncomingAdv to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv3 to intersection) <= ADV_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST