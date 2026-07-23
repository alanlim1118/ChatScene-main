"""Scenario Description:

From a top-down perspective, the ego vehicle drives straight through a four-way intersection in a city setting characterized by tall glass and concrete buildings on the corners. As the ego vehicle traverses the junction, it navigates past traffic that includes vehicles entering from the perpendicular streets on the left and right, as well as an oncoming vehicle in the opposite lane, some of which appear to be executing turns. The scene also involves pedestrians crossing the street at the marked crosswalks, creating a dynamic environment where the ego vehicle must carefully coordinate its movement to safely clear the intersection and continue along the road.

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

param EGO_SPEED = Range(3, 6)
param CROSS_TRAFFIC_SPEED = Range(3, 7)
param ONCOMING_SPEED = Range(3, 6)
param PED_SPEED = Range(0.8, 1.5)
param BRAKE_DIST = Range(8, 15)
param SAFETY_DISTANCE = 12

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoStraightBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        wait

behavior CrossingTrafficBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.CROSS_TRAFFIC_SPEED, trajectory=trajectory)
    terminate

behavior OncomingTurnBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=trajectory)
    terminate

behavior PedestrianCrossingBehavior():
    do WalkAcrossRoadBehavior(speed=globalParameters.PED_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoStartLane = egoManeuver.startLane
egoTrajectory = [egoStartLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoStartLane.centerline

# Left perpendicular crossing traffic (left turn into ego's path or straight across)
leftManeuvers = filter(lambda m: m.type in (ManeuverType.LEFT_TURN, ManeuverType.STRAIGHT),
                       filter(lambda m: abs(m.startLane.centerline.end.heading - egoStartLane.centerline.end.heading + 90 deg) < 30 deg,
                              intersection.maneuvers))
leftManeuver = Uniform(*leftManeuvers) if leftManeuvers else None
leftTrajectory = [leftManeuver.startLane, leftManeuver.connectingLane, leftManeuver.endLane] if leftManeuver else None
leftSpawnPt = new OrientedPoint in leftManeuver.startLane.centerline if leftManeuver else None

# Right perpendicular crossing traffic (right turn or straight across)
rightManeuvers = filter(lambda m: m.type in (ManeuverType.RIGHT_TURN, ManeuverType.STRAIGHT),
                        filter(lambda m: abs(m.startLane.centerline.end.heading - egoStartLane.centerline.end.heading - 90 deg) < 30 deg,
                               intersection.maneuvers))
rightManeuver = Uniform(*rightManeuvers) if rightManeuvers else None
rightTrajectory = [rightManeuver.startLane, rightManeuver.connectingLane, rightManeuver.endLane] if rightManeuver else None
rightSpawnPt = new OrientedPoint in rightManeuver.startLane.centerline if rightManeuver else None

# Oncoming vehicle in opposite lane executing a turn
oncomingManeuvers = filter(lambda m: m.type in (ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN),
                           filter(lambda m: abs(m.startLane.centerline.end.heading - egoStartLane.centerline.end.heading + 180 deg) < 30 deg,
                                  intersection.maneuvers))
oncomingManeuver = Uniform(*oncomingManeuvers) if oncomingManeuvers else None
oncomingTrajectory = [oncomingManeuver.startLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane] if oncomingManeuver else None
oncomingSpawnPt = new OrientedPoint in oncomingManeuver.startLane.centerline if oncomingManeuver else None

# Pedestrian crossing points near crosswalks of the intersection
crosswalkRegions = intersection.crosswalks if hasattr(intersection, 'crosswalks') and intersection.crosswalks else []
pedSpawnRegion = Uniform(*crosswalkRegions) if crosswalkRegions else intersection.region
pedSpawnPt = new OrientedPoint in pedSpawnRegion

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EgoStraightBehavior(egoTrajectory)

# Left crossing vehicle
if leftManeuver is not None:
    leftCar = new Car at leftSpawnPt,
        with regionContainedIn None,
        with behavior CrossingTrafficBehavior(leftTrajectory)

# Right crossing vehicle
if rightManeuver is not None:
    rightCar = new Car at rightSpawnPt,
        with regionContainedIn None,
        with behavior CrossingTrafficBehavior(rightTrajectory)

# Oncoming turning vehicle
if oncomingManeuver is not None:
    oncomingCar = new Car at oncomingSpawnPt,
        with regionContainedIn None,
        with behavior OncomingTurnBehavior(oncomingTrajectory)

# Pedestrians crossing at crosswalks
numPeds = Range(1, 3)
for i in range(numPeds):
    pedPt = new OrientedPoint in pedSpawnRegion
    ped = new Pedestrian at pedPt,
        with regionContainedIn None,
        with behavior PedestrianCrossingBehavior()

# Ensure ego starts at a reasonable distance from the intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 55

# Ensure crossing/oncoming vehicles are positioned so they interact with ego at the intersection
if leftManeuver is not None:
    require 20 <= (distance from leftSpawnPt to intersection) <= 50
if rightManeuver is not None:
    require 20 <= (distance from rightSpawnPt to intersection) <= 50
if oncomingManeuver is not None:
    require 20 <= (distance from oncomingSpawnPt to intersection) <= 50