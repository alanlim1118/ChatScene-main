"""Scenario Description:

Under dark weather conditions, the ego vehicle travels straight into an urban, multi-lane four-way intersection. As the vehicle proceeds, it suddenly brakes to yield to a pedestrian crossing the intersection from left to right, while simultaneously an adversary vehicle approaching from the left arm enters the intersection.

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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

param PED_SPEED = VerifaiRange(1.0, 1.8)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

#################################
# WEATHER                       #
#################################

param weather = Weather(
    cloudiness=100,
    precipitation=80,
    wetness=90,
    fog_density=60,
    sun_altitude=-10
)

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

behavior PedestrianCrossingBehavior(startPt, endPt):
    try:
        do WalkTowardsBehavior(target=endPt, speed=globalParameters.PED_SPEED)
    interrupt when self.position.distanceTo(endPt) < 1.0:
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary approaches from the left arm and enters the intersection (straight)
leftArmLanes = filter(lambda m:
    m.type is ManeuverType.STRAIGHT,
    egoManeuver.conflictingManeuvers)
advInitLane = Uniform(*[m.startLane for m in leftArmLanes])
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Pedestrian crosses from left to right relative to ego's direction
# Place pedestrian start point on the left side of the intersection crossing area
pedStartRegion = egoManeuver.connectingLane.leftEdge.offsetBy(-2)
pedEndRegion = egoManeuver.connectingLane.rightEdge.offsetBy(2)
pedStartPt = new OrientedPoint in pedStartRegion
pedEndPt = new OrientedPoint in pedEndRegion

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

pedestrian = new Pedestrian at pedStartPt,
    with blueprint PED_MODEL,
    with behavior PedestrianCrossingBehavior(pedStartPt, pedEndPt)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require (distance from pedestrian to intersection) < 15
terminate when (distance to egoSpawnPt) > TERM_DIST