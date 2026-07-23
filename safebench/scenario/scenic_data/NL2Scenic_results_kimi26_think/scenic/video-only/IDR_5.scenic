"""Scenario Description:

In a nighttime urban environment, the ego vehicle approaches a four-way intersection with the intent to travel straight. As it nears the junction, it encounters multiple adversary vehicles entering simultaneously. An oncoming vehicle from the opposing lane executes a left turn, crossing directly in front of the ego vehicle's path. Simultaneously, vehicles from the cross-streets enter the intersection to either turn or proceed straight, creating a convergence of traffic from multiple directions. The ego vehicle is compelled to slow down or halt to yield to these intersecting and turning vehicles, navigating carefully to prevent a collision amidst low-visibility conditions illuminated primarily by vehicle headlights.

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

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego setup: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Opposite adversary: left turn across ego's path
advOppInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.reverseManeuvers)
    ).startLane
advOppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advOppInitLane.maneuvers))
advOppTrajectory = [advOppInitLane, advOppManeuver.connectingLane, advOppManeuver.endLane]
advOppSpawnPt = new OrientedPoint in advOppInitLane.centerline

# Lateral adversary 1: from first cross-street
advLat1InitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        Uniform(*filter(lambda m:
            m.type is ManeuverType.STRAIGHT,
            egoInitLane.maneuvers)
        ).conflictingManeuvers)
    ).startLane
advLat1Maneuver = Uniform(*filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN), advLat1InitLane.maneuvers))
advLat1Trajectory = [advLat1InitLane, advLat1Maneuver.connectingLane, advLat1Maneuver.endLane]
advLat1SpawnPt = new OrientedPoint in advLat1InitLane.centerline

# Lateral adversary 2: from opposite cross-street
advLat2InitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        Uniform(*filter(lambda m:
            m.type is ManeuverType.STRAIGHT,
            advLat1InitLane.maneuvers)
        ).reverseManeuvers)
    ).startLane
advLat2Maneuver = Uniform(*filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN), advLat2InitLane.maneuvers))
advLat2Trajectory = [advLat2InitLane, advLat2Maneuver.connectingLane, advLat2Maneuver.endLane]
advLat2SpawnPt = new OrientedPoint in advLat2InitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary_opp = new Car at advOppSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advOppTrajectory)

adversary_lat1 = new Car at advLat1SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advLat1Trajectory)

adversary_lat2 = new Car at advLat2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advLat2Trajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary_opp to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary_lat1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary_lat2 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST