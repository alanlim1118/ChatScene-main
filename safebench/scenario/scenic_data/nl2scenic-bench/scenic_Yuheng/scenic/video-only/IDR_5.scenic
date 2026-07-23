"""Scenario Description:

In a nighttime urban environment captured from a top-down perspective, the ego vehicle approaches a four-way intersection with the intent to travel straight. As it nears the junction, it encounters a complex scenario involving multiple adversary vehicles entering simultaneously. An oncoming vehicle from the opposing lane executes a left turn, crossing directly in front of the ego vehicle's path. Simultaneously, vehicles from the cross-streets enter the intersection to either turn or proceed straight, creating a convergence of traffic from multiple directions. The ego vehicle is compelled to slow down or halt to yield to these intersecting and turning vehicles, navigating carefully to prevent a collision amidst the low-visibility conditions illuminated primarily by vehicle headlights.

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

# Ego goes straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 1: Oncoming vehicle making a left turn across ego's path
adv1InitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.reverseManeuvers)
    ).startLane
adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, adv1InitLane.maneuvers))
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Adversary 2: Cross-street vehicle going straight (conflicting with ego)
adv2ConflictingStraight = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers))
adv2InitLane = adv2ConflictingStraight.startLane
adv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv2InitLane.maneuvers))
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

# Adversary 3: Other cross-street vehicle turning left into ego's path
adv3ConflictingLeft = Uniform(*filter(lambda m:
        m.type is ManeuverType.LEFT_TURN,
        egoManeuver.conflictingManeuvers))
adv3InitLane = adv3ConflictingLeft.startLane
adv3Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, adv3InitLane.maneuvers))
adv3Trajectory = [adv3InitLane, adv3Maneuver.connectingLane, adv3Maneuver.endLane]
adv3SpawnPt = new OrientedPoint in adv3InitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime weather for low visibility
new Weather with precipitation=0.0, cloudiness=0.8, sunAltitude=-10.0

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with headlightsOn True

adversary1 = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv1Trajectory),
    with headlightsOn True

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv2Trajectory),
    with headlightsOn True

adversary3 = new Car at adv3SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv3Trajectory),
    with headlightsOn True

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary3 to intersection) <= ADV_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST