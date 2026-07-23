"""Scenario Description:

From a high-angle aerial perspective, a blue ego vehicle travels straight north through a four-way urban intersection surrounded by tall buildings under clear weather conditions. As the ego vehicle crosses the junction, an oncoming grey vehicle from the opposing northern arm executes a right turn onto the western cross street. Simultaneously, traffic enters from the western left arm: a red vehicle proceeds straight across the intersection towards the east, while a grey vehicle executes a left turn, heading south. The road features clear white lane markings, pedestrian crosswalks, and a bus stop zone marked on the pavement to the left of the ego vehicle's path.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(6, 10)

EGO_INIT_DIST = [25, 35]
ADV_INIT_DIST = [20, 30]

TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, 15):
        take SetBrakeAction(0.8)
    interrupt when withinDistanceToAnyObjs(self, 5):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: traveling straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 1: Oncoming vehicle from opposite arm making a RIGHT turn
# The reverse maneuver of ego's straight gives us the opposing straight lane
adv1InitLane = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers)).startLane
adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, adv1InitLane.maneuvers))
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Adversary 2: From the western/left arm going STRAIGHT (eastbound)
# Find lanes that are left of ego's incoming lane at this intersection
leftArmLanes = filter(lambda l: 
    abs(relativeHeading(l.centerline.start, egoInitLane.centerline.start) - 90 deg) < 30 deg,
    intersection.incomingLanes)
adv2InitLane = Uniform(*leftArmLanes) if leftArmLanes else Uniform(*intersection.incomingLanes)
adv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv2InitLane.maneuvers))
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

# Adversary 3: From the same western/left arm making a LEFT turn (heading south)
adv3InitLane = adv2InitLane
adv3Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, adv3InitLane.maneuvers))
adv3Trajectory = [adv3InitLane, adv3Maneuver.connectingLane, adv3Maneuver.endLane]
adv3SpawnPt = new OrientedPoint in adv3InitLane.centerline,
    with offset (-3, 0)  # Slight lateral offset to avoid spawn collision with adv2

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue ego vehicle going straight
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

# Grey oncoming vehicle turning right
adversary1 = new Car at adv1SpawnPt,
    with blueprint ADV_MODEL,
    with color (0.5, 0.5, 0.5),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv1Trajectory)

# Red vehicle from west going straight east
adversary2 = new Car at adv2SpawnPt,
    with blueprint ADV_MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv2Trajectory)

# Grey vehicle from west turning left (southbound)
adversary3 = new Car at adv3SpawnPt,
    with blueprint ADV_MODEL,
    with color (0.5, 0.5, 0.5),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv3Trajectory)

# Distance constraints for realistic intersection approach
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary3 to intersection) <= ADV_INIT_DIST[1]

# Ensure adversaries are on distinct lanes where possible
require (distance from adversary2 to adversary3) > 2

# Termination condition
terminate when (distance from ego to egoSpawnPt) > TERM_DIST