"""Scenario Description:

In a four-way intersection, an ego vehicle and an adversary vehicle are driving side-by-side in adjacent lanes, both executing a straight-through maneuver as indicated by solid upward-pointing arrows. The diagram features a pink vehicle and an adversary blue vehicle positioned closely together at the approach to the intersection, with the adversary vehicle overlapping into the ego vehicle's driving lane, suggesting a lane encroachment. Dashed trajectory lines surround the vehicles, illustrating potential left and right turn options for traffic, but the core scenario focuses on the two vehicles traveling parallel to each other with the adversary intruding into the ego vehicle's space while proceeding straight.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.tesla.model3'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [20, 25]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 3
TERM_DIST = 70

# Lateral offset for adversary to encroach into ego's lane
ENCROACH_OFFSET = Range(-0.8, -0.3)

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

behavior AdversaryEncroachBehavior(trajectory, lateralOffset):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory, lateral_offset=lateralOffset)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
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

# Adversary is in an adjacent lane going straight (same direction as ego)
# Find lanes that have straight maneuvers parallel to ego's straight maneuver
adjacentStraightManeuvers = filter(lambda m: 
    m.type is ManeuverType.STRAIGHT and m.startLane is not egoInitLane,
    egoInitLane.leftLane.maneuvers if egoInitLane.leftLane else []
) if egoInitLane.leftLane else filter(lambda m:
    m.type is ManeuverType.STRAIGHT and m.startLane is not egoInitLane,
    egoInitLane.rightLane.maneuvers if egoInitLane.rightLane else []
)

advManeuver = Uniform(*adjacentStraightManeuvers) if adjacentStraightManeuvers else egoManeuver
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with color (1.0, 0.75, 0.8)  # Pink vehicle

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryEncroachBehavior(advTrajectory, ENCROACH_OFFSET),
    with color (0.2, 0.4, 0.9)  # Blue vehicle

# Both vehicles should be at similar distances from the intersection (side-by-side)
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Ensure adversary is in an adjacent lane to ego
require (distance from ego to adversary) < 6

terminate when (distance to egoSpawnPt) > TERM_DIST