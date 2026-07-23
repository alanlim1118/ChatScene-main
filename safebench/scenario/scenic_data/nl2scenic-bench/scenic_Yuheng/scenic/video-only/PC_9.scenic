"""Scenario Description:

The ego vehicle proceeds straight through a four-way intersection in an urban environment characterized by heavy mist and significantly reduced visibility. As the vehicle navigates the junction, which is marked with stop lines and crosswalks, it encounters dynamic traffic from multiple directions. An adversary vehicle enters the intersection from the left cross-street, while another vehicle from the opposing lane also moves into the junction, potentially to turn or proceed straight. Simultaneously, a pedestrian is present near the crosswalk, necessitating that the ego vehicle exercise heightened caution to safely pass the intersecting vehicular traffic and the pedestrian in the foggy conditions.

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
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_LEFT_INIT_DIST = [15, 25]
param ADV_LEFT_SPEED = VerifaiRange(6, 9)

ADV_OPP_INIT_DIST = [15, 25]
param ADV_OPP_SPEED = VerifaiRange(6, 9)

PED_SPEED = Range(0.8, 1.5)
PED_STOP_DIST = Range(1, 3)

param SAFETY_DIST = VerifaiRange(12, 20)
CRASH_DIST = 4
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior PedestrianCrossAndStopBehavior(actor_ref, speed, stop_ref, stop_dist):
    do CrossingBehavior(actor_ref, speed) until (distance from self to stop_ref <= stop_dist)
    take SetWalkingSpeedAction(0)

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

# Ego goes straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary from left cross-street going straight
leftConflictingManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers)
advLeftInitLane = Uniform(*[m.startLane for m in leftConflictingManeuvers])
advLeftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advLeftInitLane.maneuvers))
advLeftTrajectory = [advLeftInitLane, advLeftManeuver.connectingLane, advLeftManeuver.endLane]
advLeftSpawnPt = new OrientedPoint in advLeftInitLane.centerline

# Adversary from opposing lane (reverse maneuver direction), going straight or turning
oppManeuvers = filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN), egoManeuver.reverseManeuvers)
advOppInitLane = Uniform(*[m.startLane for m in oppManeuvers])
advOppManeuver = Uniform(*filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN), advOppInitLane.maneuvers))
advOppTrajectory = [advOppInitLane, advOppManeuver.connectingLane, advOppManeuver.endLane]
advOppSpawnPt = new OrientedPoint in advOppInitLane.centerline

# Pedestrian near crosswalk on ego's end lane
endLaneCenterline = egoManeuver.endLane.centerline
pedSpawnRegion = Region.fromPolyline(endLaneCenterline.points[:max(1, len(endLaneCenterline.points)//3)])
pedSpawnPt = new OrientedPoint in pedSpawnRegion,
    with heading egoManeuver.endLane.centerline.end.heading + 90 deg
pedStopRef = egoManeuver.endLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set heavy fog weather
param weather = Weather(preset='HeavyFog')

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversaryLeft = new Car at advLeftSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_LEFT_SPEED, trajectory=advLeftTrajectory)

adversaryOpp = new Car at advOppSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_OPP_SPEED, trajectory=advOppTrajectory)

pedestrian = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossAndStopBehavior(ego, PED_SPEED, pedStopRef, PED_STOP_DIST)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_LEFT_INIT_DIST[0] <= (distance from adversaryLeft to intersection) <= ADV_LEFT_INIT_DIST[1]
require ADV_OPP_INIT_DIST[0] <= (distance from adversaryOpp to intersection) <= ADV_OPP_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST