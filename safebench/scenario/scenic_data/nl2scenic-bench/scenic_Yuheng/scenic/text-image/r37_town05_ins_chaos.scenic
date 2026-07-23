"""Scenario Description:

This top-down simulation view illustrates a complex urban traffic scenario at a four-way intersection flanked by city buildings with an overpass at the north end, a parking lot with numerous parked cars to the southwest, and a green park space to the southeast. The ego vehicle is positioned in the southern approach lane, executing a left turn into the intersection, while a leading vehicle directly ahead in the same lane initiates a right-hand turn. Cross traffic is present on the perpendicular roads, with two adversarial vehicles approaching from the west and two others moving from the right side toward the junction. Additionally, two oncoming vehicles are visible near the northern overpass, traveling south toward the intersection, and various colored trajectory lines overlay the road surface to indicate the diverse turning and straight paths of the agents navigating the scene.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(4, 8)
param OPT_BRAKE_DIST = Range(8, 15)
param OPT_LEAD_SPEED = Range(3, 5)
param OPT_ONCOMING_SPEED = Range(5, 9)

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior LeadVehicleBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_LEAD_SPEED, leadTrajectory)
    terminate

behavior AdvApproachBehavior(trajectory):
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, trajectory)
    terminate

behavior OncomingBehavior(trajectory):
    do FollowTrajectoryBehavior(globalParameters.OPT_ONCOMING_SPEED, trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection (preferably signalized for realistic traffic flow)
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: Left turn from southern approach
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Leading vehicle: Right turn from same lane as ego
leadManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.startLane is egoInitLane, intersection.maneuvers))
leadTrajectory = [leadManeuver.startLane, leadManeuver.connectingLane, leadManeuver.endLane]
leadSpawnPt = new OrientedPoint in egoInitLane.centerline,
    ahead of egoSpawnPt by Range(15, 25)

# West adversarial vehicles (approaching from west, i.e., from the left relative to ego's perspective)
westManeuvers = filter(lambda m: m.startLane.road.heading - egoInitLane.road.heading > 45 deg and m.startLane.road.heading - egoInitLane.road.heading < 135 deg, intersection.maneuvers)
westManeuverList = list(westManeuvers)
westMan1 = Uniform(*westManeuverList) if westManeuverList else egoManeuver
westMan2 = Uniform(*westManeuverList) if westManeuverList else egoManeuver
westTraj1 = [westMan1.startLane, westMan1.connectingLane, westMan1.endLane]
westTraj2 = [westMan2.startLane, westMan2.connectingLane, westMan2.endLane]
westSpawn1 = new OrientedPoint in westMan1.startLane.centerline
westSpawn2 = new OrientedPoint in westMan2.startLane.centerline,
    behind westSpawn1 by Range(15, 30)

# East/right-side adversarial vehicles (approaching from east/right relative to ego)
eastManeuvers = filter(lambda m: m.startLane.road.heading - egoInitLane.road.heading > 225 deg or m.startLane.road.heading - egoInitLane.road.heading < -45 deg, intersection.maneuvers)
eastManeuverList = list(eastManeuvers)
eastMan1 = Uniform(*eastManeuverList) if eastManeuverList else egoManeuver
eastMan2 = Uniform(*eastManeuverList) if eastManeuverList else egoManeuver
eastTraj1 = [eastMan1.startLane, eastMan1.connectingLane, eastMan1.endLane]
eastTraj2 = [eastMan2.startLane, eastMan2.connectingLane, eastMan2.endLane]
eastSpawn1 = new OrientedPoint in eastMan1.startLane.centerline
eastSpawn2 = new OrientedPoint in eastMan2.startLane.centerline,
    behind eastSpawn1 by Range(15, 30)

# Oncoming vehicles from north (near overpass), traveling south toward intersection
northManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT and abs(m.startLane.road.heading - egoInitLane.road.heading + 180 deg) < 30 deg, intersection.maneuvers)
northManeuverList = list(northManeuvers)
northMan1 = Uniform(*northManeuverList) if northManeuverList else egoManeuver
northMan2 = Uniform(*northManeuverList) if northManeuverList else egoManeuver
northTraj1 = [northMan1.startLane, northMan1.connectingLane, northMan1.endLane]
northTraj2 = [northMan2.startLane, northMan2.connectingLane, northMan2.endLane]
northSpawn1 = new OrientedPoint in northMan1.startLane.centerline
northSpawn2 = new OrientedPoint in northMan2.startLane.centerline,
    behind northSpawn1 by Range(20, 35)

#################################
# SCENARIO SPECIFICATION        #
#################################

require monitor TrafficLights()

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

leadCar = new Car at leadSpawnPt,
    with heading leadSpawnPt.heading,
    with regionContainedIn None,
    with behavior LeadVehicleBehavior()

advWest1 = new Car at westSpawn1,
    with heading westSpawn1.heading,
    with regionContainedIn None,
    with behavior AdvApproachBehavior(westTraj1)

advWest2 = new Car at westSpawn2,
    with heading westSpawn2.heading,
    with regionContainedIn None,
    with behavior AdvApproachBehavior(westTraj2)

advEast1 = new Car at eastSpawn1,
    with heading eastSpawn1.heading,
    with regionContainedIn None,
    with behavior AdvApproachBehavior(eastTraj1)

advEast2 = new Car at eastSpawn2,
    with heading eastSpawn2.heading,
    with regionContainedIn None,
    with behavior AdvApproachBehavior(eastTraj2)

oncoming1 = new Car at northSpawn1,
    with heading northSpawn1.heading,
    with regionContainedIn None,
    with behavior OncomingBehavior(northTraj1)

oncoming2 = new Car at northSpawn2,
    with heading northSpawn2.heading,
    with regionContainedIn None,
    with behavior OncomingBehavior(northTraj2)

# Ensure ego starts at reasonable distance from intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 60
# Ensure leading vehicle is ahead of ego but not too far
require 15 <= (distance from egoSpawnPt to leadSpawnPt) <= 30
# Ensure adversarial vehicles are at plausible distances
require 20 <= (distance from westSpawn1 to intersection) <= 60
require 20 <= (distance from eastSpawn1 to intersection) <= 60
require 30 <= (distance from northSpawn1 to intersection) <= 70