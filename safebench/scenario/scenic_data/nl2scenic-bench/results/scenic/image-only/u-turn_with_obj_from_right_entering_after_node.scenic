"""Scenario Description:

A blue vehicle executes a U-turn at a four-way intersection, curving from the northbound direction back towards the south. After the ego vehicle completes its maneuver through the intersection center, a purple adversary object enters the bottom lane from the right side of the intersection, moving towards the same lane the ego is now traveling in.

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

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(6, 9)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
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

behavior AdversaryDelayedBehavior(trajectory, triggerPoint):
    try:
        do WaitUntilBehavior(lambda: distance to triggerPoint < 5)
        do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego performs a U-turn: find incoming lane with U-turn maneuver
egoInitLane = Uniform(*filter(lambda l: 
    any(m.type is ManeuverType.U_TURN for m in l.maneuvers),
    intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.U_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary enters from the right relative to ego's initial heading,
# targeting the same lane that ego ends up in after U-turn
advEntryLane = egoManeuver.endLane
advInitLane = Uniform(*filter(lambda l:
    l is not egoInitLane and l is not advEntryLane,
    intersection.incomingLanes))
advManeuverCandidates = filter(lambda m: 
    m.endLane is advEntryLane or m.connectingLane is advEntryLane,
    advInitLane.maneuvers)
advManeuver = Uniform(*advManeuverCandidates) if advManeuverCandidates else Uniform(*advInitLane.maneuvers)
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Trigger point: center of intersection where ego completes U-turn
triggerRegion = CircularRegion(intersection.position, 10)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color (0.5, 0, 0.5),
    with behavior AdversaryDelayedBehavior(advTrajectory, triggerRegion)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST