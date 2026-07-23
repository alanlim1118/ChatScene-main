"""Scenario Description:

The ego vehicle approaches a four-way urban intersection on a multi-lane road under clear weather conditions and prepares to turn left. Upon entering the intersection, the ego vehicle yields to an oncoming adversary vehicle traveling straight from the opposite direction, as well as to three adversary vehicles crossing from the right arm. As the vehicular traffic clears, a pedestrian is seen navigating through the intersection, crossing from the ego vehicle's right side toward the left, allowing the ego vehicle to complete its left turn and proceed along the cross street.

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
PED_MODEL = 'walker.pedestrian.0001'

EGO_INIT_DIST = [25, 35]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_ONCOMING_DIST = [20, 30]
ADV_RIGHT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(7, 11)

PED_DELAY = VerifaiRange(3.0, 6.0)
param PED_SPEED = VerifaiRange(1.0, 1.8)

param SAFETY_DIST = VerifaiRange(10, 18)
CRASH_DIST = 4
TERM_DIST = 80

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

behavior DelayedPedestrianBehavior(trajectory, delay):
    wait delay
    do FollowTrajectoryBehavior(target_speed=globalParameters.PED_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: left turn at 4-way intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Oncoming adversary: straight from opposite direction
oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oncomingInitLane = oncomingManeuver.startLane
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingInitLane.centerline

# Right-arm adversaries: crossing from ego's right (straight across intersection)
# The right arm relative to ego's incoming lane corresponds to lanes whose straight
# maneuvers conflict with ego's left turn from the right side.
rightArmLanes = filter(lambda lane:
    any(m.type is ManeuverType.STRAIGHT for m in lane.maneuvers) and
    lane is not egoInitLane and
    lane is not oncomingInitLane,
    intersection.incomingLanes)

# We pick one representative right-arm lane; all three adversaries spawn along it
rightArmLane = Uniform(*rightArmLanes)
rightArmManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightArmLane.maneuvers))
rightArmTrajectory = [rightArmLane, rightArmManeuver.connectingLane, rightArmManeuver.endLane]

rightAdvSpawnPts = [new OrientedPoint in rightArmLane.centerline for _ in range(3)]

# Pedestrian: crosses from ego's right to left through the intersection
# Use the crosswalk or sidewalk region near the intersection center
pedCrossRegion = intersection.region
pedStartPt = new OrientedPoint in pedCrossRegion,
    with heading egoSpawnPt.heading + Range(-1.2, -0.8)
pedEndPt = new OrientedPoint in pedCrossRegion,
    with heading egoSpawnPt.heading + Range(1.8, 2.2),
    visible from pedStartPt
pedTrajectory = [pedStartPt, pedEndPt]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

oncomingAdv = new Car at oncomingSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=oncomingTrajectory)

rightAdv1 = new Car at rightAdvSpawnPts[0],
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightArmTrajectory)

rightAdv2 = new Car at rightAdvSpawnPts[1],
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightArmTrajectory)

rightAdv3 = new Car at rightAdvSpawnPts[2],
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightArmTrajectory)

pedestrian = new Pedestrian at pedStartPt,
    with blueprint PED_MODEL,
    with behavior DelayedPedestrianBehavior(pedTrajectory, PED_DELAY)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_ONCOMING_DIST[0] <= (distance from oncomingAdv to intersection) <= ADV_ONCOMING_DIST[1]
require all(ADV_RIGHT_DIST[0] <= (distance from adv to intersection) <= ADV_RIGHT_DIST[1] for adv in [rightAdv1, rightAdv2, rightAdv3])
require (distance from pedestrian to intersection) < 15

terminate when (distance from ego to egoSpawnPt) > TERM_DIST