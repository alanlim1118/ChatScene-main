"""Scenario Description:

In a top-down view of a four-way intersection, the ego vehicle attempts to proceed straight through the junction but remains positioned in the center lane. It encounters a complex traffic situation involving multiple adversary vehicles, which approach and cross from various arms of the intersection. Additionally, pedestrians are visible at the corners of the junction. Due to these intersecting vehicle movements and the presence of pedestrians, the ego vehicle is required to yield, waiting in the middle of the intersection to carefully monitor the cross-traffic and pedestrian paths before it can safely resume its straight trajectory.

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

EGO_INIT_DIST = [15, 25]
param EGO_SPEED = VerifaiRange(5, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [10, 20]
param ADV_SPEED = VerifaiRange(5, 10)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 5
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

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

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries sampled from maneuvers that conflict with the ego's straight path
adv1Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv1Lane = adv1Maneuver.startLane
adv1Trajectory = [adv1Lane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1Lane.centerline

adv2Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv2Lane = adv2Maneuver.startLane
adv2Trajectory = [adv2Lane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2Lane.centerline

adv3Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv3Lane = adv3Maneuver.startLane
adv3Trajectory = [adv3Lane, adv3Maneuver.connectingLane, adv3Maneuver.endLane]
adv3SpawnPt = new OrientedPoint in adv3Lane.centerline

# Pedestrians positioned at the four corners of the intersection
ped1Pos = new OrientedPoint at intersection.center offset by (-4, -4)
ped2Pos = new OrientedPoint at intersection.center offset by (-4, 4)
ped3Pos = new OrientedPoint at intersection.center offset by (4, -4)
ped4Pos = new OrientedPoint at intersection.center offset by (4, 4)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv1Trajectory)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv2Trajectory)

adversary3 = new Car at adv3SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv3Trajectory)

ped1 = new Pedestrian at ped1Pos,
    with behavior WaitBehavior()

ped2 = new Pedestrian at ped2Pos,
    with behavior WaitBehavior()

ped3 = new Pedestrian at ped3Pos,
    with behavior WaitBehavior()

ped4 = new Pedestrian at ped4Pos,
    with behavior WaitBehavior()

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary3 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST