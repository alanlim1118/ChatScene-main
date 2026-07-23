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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV1_SPEED = VerifaiRange(5, 9)
param ADV2_SPEED = VerifaiRange(5, 9)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

PED_SPEED = 1.0

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

behavior AdvBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

behavior PedestrianBehavior():
    take SetWalkingSpeedAction(PED_SPEED)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego setup: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Opposing straight maneuver for reference
oppositeStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oppositeLane = oppositeStraightManeuver.startLane

# Adversary 1: from left cross-street (lateral incoming lane)
lateralLanes = [l for l in intersection.incomingLanes if l is not egoInitLane and l is not oppositeLane]
adv1InitLane = Uniform(*lateralLanes)
adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv1InitLane.maneuvers))
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Adversary 2: from opposing lane, either straight or left turn
adv2InitLane = oppositeLane
adv2Maneuver = Uniform(*filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN), adv2InitLane.maneuvers))
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

# Pedestrian: near the crosswalk on the far side of the intersection
endLanePt = new OrientedPoint at egoManeuver.endLane.rightEdge.start,
    with heading egoInitLane.centerline.end.heading - 180 deg
pedSpawnPt = new OrientedPoint ahead of endLanePt by -6

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(adv1Trajectory, globalParameters.ADV1_SPEED)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(adv2Trajectory, globalParameters.ADV2_SPEED)

pedestrian = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading,
    with behavior PedestrianBehavior()

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST