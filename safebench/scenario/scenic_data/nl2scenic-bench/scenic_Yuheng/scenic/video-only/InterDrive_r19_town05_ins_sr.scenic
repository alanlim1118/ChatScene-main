"""Scenario Description:

The ego vehicle, a blue car, begins at a stop line at the bottom of a four-way intersection situated between a tall brown brick building on the left and a white high-rise on the right. As the ego vehicle accelerates straight through the intersection, a red adversary vehicle approaching from the right cross-street executes a right turn, merging into the lane directly ahead of the ego vehicle. The ego vehicle continues its forward path, now following the red car which has established itself as the lead vehicle in the same lane, as they both proceed northward past a grassy lot containing construction equipment and a large Coca-Cola sign.

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
ADV_MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
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

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego starts from an incoming lane going straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary comes from the right cross-street relative to ego's direction
# and performs a RIGHT TURN into ego's end lane (merging ahead of ego)
rightIncomingLanes = filter(lambda m:
    m.type is ManeuverType.STRAIGHT,
    egoManeuver.conflictingManeuvers
)
advInitLane = Uniform(*map(lambda m: m.startLane, rightIncomingLanes))
advManeuver = Uniform(*filter(lambda m: 
    m.type is ManeuverType.RIGHT_TURN and m.endLane is egoManeuver.endLane,
    advInitLane.maneuvers
))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "0,0,255",
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color "255,0,0",
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST