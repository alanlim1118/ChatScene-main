"""Scenario Description:

In a dense urban area characterized by high-rise buildings and a parking lot visible in the upper left, the ego vehicle proceeds straight through a multi-lane four-way intersection. The scenario depicts a highly complex conflict involving multiple adversaries with intersecting trajectories visualized by colored lines. Specifically, a purple vehicle from the same approach arm executes a left turn, while a green vehicle from the opposing arm, a blue vehicle from the left arm, and a yellow vehicle from the right arm all attempt to pass straight through the intersection, creating a crowded and potentially hazardous traffic situation where multiple paths cross.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

PURPLE_INIT_DIST = [15, 20]
param PURPLE_SPEED = VerifaiRange(6, 9)

GREEN_INIT_DIST = [15, 20]
param GREEN_SPEED = VerifaiRange(7, 10)

BLUE_INIT_DIST = [15, 20]
param BLUE_SPEED = VerifaiRange(7, 10)

YELLOW_INIT_DIST = [15, 20]
param YELLOW_SPEED = VerifaiRange(7, 10)

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

# Ego vehicle: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Purple vehicle: same approach arm, left turn
purpleInitLane = Uniform(*filter(lambda l: l.road is egoInitLane.road and any(m.type is ManeuverType.LEFT_TURN for m in l.maneuvers), intersection.incomingLanes))
purpleManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, purpleInitLane.maneuvers))
purpleTrajectory = [purpleInitLane, purpleManeuver.connectingLane, purpleManeuver.endLane]
purpleSpawnPt = new OrientedPoint in purpleInitLane.centerline

# Green vehicle: opposing arm, straight
greenManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
greenInitLane = greenManeuver.startLane
greenTrajectory = [greenInitLane, greenManeuver.connectingLane, greenManeuver.endLane]
greenSpawnPt = new OrientedPoint in greenInitLane.centerline

# Blue and Yellow vehicles: left and right arms, straight
# Identify the four unique incoming roads
incomingRoads = []
for lane in intersection.incomingLanes:
    if lane.road not in incomingRoads:
        incomingRoads.append(lane.road)

# Determine lateral roads (exclude ego road and opposing road)
opposingRoad = greenInitLane.road
lateralRoads = [r for r in incomingRoads if r is not egoInitLane.road and r is not opposingRoad]
blueRoad = lateralRoads[0]
yellowRoad = lateralRoads[1]

# Blue vehicle: left arm, straight
blueInitLane = Uniform(*filter(lambda l: l.road is blueRoad and any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), intersection.incomingLanes))
blueManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, blueInitLane.maneuvers))
blueTrajectory = [blueInitLane, blueManeuver.connectingLane, blueManeuver.endLane]
blueSpawnPt = new OrientedPoint in blueInitLane.centerline

# Yellow vehicle: right arm, straight
yellowInitLane = Uniform(*filter(lambda l: l.road is yellowRoad and any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), intersection.incomingLanes))
yellowManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, yellowInitLane.maneuvers))
yellowTrajectory = [yellowInitLane, yellowManeuver.connectingLane, yellowManeuver.endLane]
yellowSpawnPt = new OrientedPoint in yellowInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

purple = new Car at purpleSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.PURPLE_SPEED, trajectory=purpleTrajectory)

green = new Car at greenSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.GREEN_SPEED, trajectory=greenTrajectory)

blue = new Car at blueSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.BLUE_SPEED, trajectory=blueTrajectory)

yellow = new Car at yellowSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.YELLOW_SPEED, trajectory=yellowTrajectory)

# Distance requirements
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require PURPLE_INIT_DIST[0] <= (distance from purple to intersection) <= PURPLE_INIT_DIST[1]
require GREEN_INIT_DIST[0] <= (distance from green to intersection) <= GREEN_INIT_DIST[1]
require BLUE_INIT_DIST[0] <= (distance from blue to intersection) <= BLUE_INIT_DIST[1]
require YELLOW_INIT_DIST[0] <= (distance from yellow to intersection) <= YELLOW_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST