"""Scenario Description:

In a dense urban canyon formed by tall skyscrapers, the ego vehicle travels along a city street approaching a signalized four-way intersection under clear daylight conditions. The ego vehicle follows a red lead vehicle that is executing a left turn through the junction. As the ego vehicle enters the intersection to complete the same left turn, it must navigate around adversary vehicles that are simultaneously entering from the opposing lane and the right-hand cross-street to proceed straight. The scenario requires the ego vehicle to maintain a safe following distance behind the red car while carefully monitoring the conflicting straight-moving traffic to ensure a safe passage through the intersection.

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
LEAD_MODEL = 'vehicle.tesla.model3'

EGO_INIT_DIST = [25, 35]
LEAD_FOLLOW_DIST = [8, 12]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_OPP_INIT_DIST = [15, 25]
ADV_RIGHT_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory, leadVehicle):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when (distance to leadVehicle) < globalParameters.LEAD_FOLLOW_DIST[0]:
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego and lead vehicle share the same incoming lane and left turn maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

leadSpawnPt = new OrientedPoint in egoInitLane.centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary from opposing lane going straight
oppInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.reverseManeuvers)
    ).startLane
oppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, oppInitLane.maneuvers))
oppTrajectory = [oppInitLane, oppManeuver.connectingLane, oppManeuver.endLane]
oppSpawnPt = new OrientedPoint in oppInitLane.centerline

# Adversary from right-hand cross-street going straight
rightInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers)
    ).startLane
rightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightInitLane.maneuvers))
rightTrajectory = [rightInitLane, rightManeuver.connectingLane, rightManeuver.endLane]
rightSpawnPt = new OrientedPoint in rightInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

leadCar = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with color (0.8, 0.0, 0.0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, leadCar)

adversaryOpp = new Car at oppSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=oppTrajectory)

adversaryRight = new Car at rightSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightTrajectory)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require LEAD_FOLLOW_DIST[0] <= (distance from ego to leadCar) <= LEAD_FOLLOW_DIST[1]
require ADV_OPP_INIT_DIST[0] <= (distance from adversaryOpp to intersection) <= ADV_OPP_INIT_DIST[1]
require ADV_RIGHT_INIT_DIST[0] <= (distance from adversaryRight to intersection) <= ADV_RIGHT_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST