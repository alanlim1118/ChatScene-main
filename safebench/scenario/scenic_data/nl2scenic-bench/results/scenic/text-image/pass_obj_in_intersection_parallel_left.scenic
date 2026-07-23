"""Scenario Description:

Ego vehicle (blue) travels straight through a four-way intersection in the right lane while an adversarial vehicle (pink) in the adjacent left lane also proceeds straight, parallel to the ego. Both vehicles pass through the junction simultaneously without turning, illustrating a parallel passing maneuver where the ego passes an object on its left.

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

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [20, 30]
param ADV_SPEED = VerifaiRange(8, 12)

param SAFETY_DIST = VerifaiRange(8, 15)
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

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select ego lane and straight maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary must be in the adjacent left lane going straight (parallel)
# Find lanes that are left-adjacent to egoInitLane and have a straight maneuver
leftAdjacentLanes = filter(lambda l: 
    l is not egoInitLane and 
    any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers),
    egoInitLane.leftLanes if hasattr(egoInitLane, 'leftLanes') else []
)

# Fallback: if leftLanes attribute not available, use conflicting straight maneuvers' start lanes
# that are geometrically to the left of ego
advCandidateLanes = leftAdjacentLanes if leftAdjacentLanes else filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
            .conflictingManeuvers
    ).startLane

advInitLane = Uniform(*advCandidateLanes)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Ensure adversary is to the left of ego
require relative position of adversary is left of ego

# Ensure both vehicles are at similar distances from intersection (parallel start)
require abs((distance to intersection) - (distance from adversary to intersection)) < 5

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST