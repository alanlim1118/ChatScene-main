"""Scenario Description:

A top-down diagram of a four-way intersection shows two vehicles entering from the bottom in parallel lanes. The dark blue vehicle in the left lane is proceeding straight through the intersection, indicated by a straight blue arrow pointing north. To its right, a pink vehicle is executing a U-turn maneuver, illustrated by a curved purple arrow that loops from the northbound direction back towards the south.

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

MODEL_BLUE = 'vehicle.lincoln.mkz_2017'
MODEL_PINK = 'vehicle.tesla.model3'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [20, 25]
param ADV_SPEED = VerifaiRange(5, 8)

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

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Dark blue vehicle (ego) in the left lane going straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Pink vehicle (adversary) in the right lane making a U-turn
# The adversary starts in a lane to the right of ego's lane at the same intersection
advInitLane = Uniform(*filter(lambda lane:
        lane is not egoInitLane and
        any(m.type is ManeuverType.U_TURN for m in lane.maneuvers),
        intersection.incomingLanes))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.U_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL_BLUE,
    with color (0, 0, 139),
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL_PINK,
    with color (255, 192, 203),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST