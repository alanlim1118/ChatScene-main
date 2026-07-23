"""Scenario Description:

This top-down aerial view captures an urban traffic scenario at a four-way intersection, featuring a multi-space parking lot to the bottom left containing several stationary vehicles including a grey SUV, a yellow SUV, and a red sedan. At the intersection's left approach, the ego vehicle, a red car, is positioned in the upper lane at the stop line, while directly adjacent in the lower lane is the primary adversarial vehicle, a blue car, which is following a cyan trajectory path to make a right turn onto the vertical cross street. Trailing closely behind the ego vehicle in the upper lane is a green car with a trajectory indicating a right turn, and behind the primary adversarial vehicle in the lower lane is a yellow car with a long yellow trajectory path extending straight across the intersection. The scene is illuminated by daylight, casting shadows from the traffic poles and vehicles, and includes standard road markings such as double yellow lines, crosswalks, and lane dividers.

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
PARKED_GREY_SUV_MODEL = "vehicle.nissan.patrol"
PARKED_YELLOW_SUV_MODEL = "vehicle.jeep.wrangler_rubicon"
PARKED_RED_SEDAN_MODEL = "vehicle.tesla.model3"
GREEN_CAR_MODEL = "vehicle.toyota.prius"
YELLOW_CAR_MODEL = "vehicle.audi.tt"
ADV_BLUE_CAR_MODEL = "vehicle.bmw.grandtourer"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_GREEN_SPEED = Range(3, 6)
param OPT_YELLOW_SPEED = Range(5, 8)
param OPT_BRAKE_DISTANCE = Range(5, 10)
param OPT_FOLLOW_DISTANCE = Range(8, 15)

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent, 100):
            setClosestTrafficLightStatus(AdvAgent, "green")
        if withinDistanceToTrafficLight(GreenCar, 100):
            setClosestTrafficLightStatus(GreenCar, "green")
        if withinDistanceToTrafficLight(YellowCar, 100):
            setClosestTrafficLightStatus(YellowCar, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 3 seconds
        abort
    terminate

behavior AdvRightTurnBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

behavior GreenRightTurnBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_FOLLOW_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_GREEN_SPEED, greenTrajectory)
    terminate

behavior YellowStraightBehavior():
    do WaitBehavior() until (distance from self to AdvAgent) < globalParameters.OPT_FOLLOW_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_YELLOW_SPEED, yellowTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: upper lane on left approach, going straight
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversarial blue car: lower lane on left approach, right turn
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Green car: trailing ego in upper lane, right turn
greenManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
greenTrajectory = [greenManeuver.startLane, greenManeuver.connectingLane, greenManeuver.endLane]
greenInitLane = greenManeuver.startLane
greenSpawnPt = new OrientedPoint in greenInitLane.centerline

# Yellow car: trailing adv in lower lane, going straight
yellowManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advManeuver.conflictingManeuvers))
yellowTrajectory = [yellowManeuver.startLane, yellowManeuver.connectingLane, yellowManeuver.endLane]
yellowInitLane = yellowManeuver.startLane
yellowSpawnPt = new OrientedPoint in yellowInitLane.centerline

# Parking lot region (bottom-left relative to intersection)
parkingRegion = RectangularRegion(
    position=intersection.position.offsetBy(-30, -30),
    width=25, height=20,
    heading=intersection.heading
)

parkedGreyPt = new OrientedPoint in parkingRegion, with heading Uniform(0 deg, 90 deg, 180 deg, 270 deg)
parkedYellowPt = new OrientedPoint in parkingRegion, with heading Uniform(0 deg, 90 deg, 180 deg, 270 deg)
parkedRedPt = new OrientedPoint in parkingRegion, with heading Uniform(0 deg, 90 deg, 180 deg, 270 deg)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle - red car in upper lane at stop line
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color "red",
    with behavior EgoBehavior()

# Primary adversarial vehicle - blue car in lower lane making right turn
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_BLUE_CAR_MODEL,
    with color "blue",
    with behavior AdvRightTurnBehavior()

# Green car trailing ego in upper lane, turning right
GreenCar = new Car at greenSpawnPt,
    with heading greenSpawnPt.heading,
    with regionContainedIn None,
    with blueprint GREEN_CAR_MODEL,
    with color "green",
    with behavior GreenRightTurnBehavior()

# Yellow car trailing adv in lower lane, going straight
YellowCar = new Car at yellowSpawnPt,
    with heading yellowSpawnPt.heading,
    with regionContainedIn None,
    with blueprint YELLOW_CAR_MODEL,
    with color "yellow",
    with behavior YellowStraightBehavior()

# Stationary parked vehicles in parking lot
ParkedGreySUV = new Car at parkedGreyPt,
    with heading parkedGreyPt.heading,
    with regionContainedIn parkingRegion,
    with blueprint PARKED_GREY_SUV_MODEL,
    with color "grey",
    with behavior WaitBehavior()

ParkedYellowSUV = new Car at parkedYellowPt,
    with heading parkedYellowPt.heading,
    with regionContainedIn parkingRegion,
    with blueprint PARKED_YELLOW_SUV_MODEL,
    with color "yellow",
    with behavior WaitBehavior()

ParkedRedSedan = new Car at parkedRedPt,
    with heading parkedRedPt.heading,
    with regionContainedIn parkingRegion,
    with blueprint PARKED_RED_SEDAN_MODEL,
    with color "red",
    with behavior WaitBehavior()

# Constraints
require monitor TrafficLights()
require distance from egoSpawnPt to advSpawnPt <= 10  # Adjacent lanes
require distance from greenSpawnPt to egoSpawnPt >= 8  # Trailing behind ego
require distance from yellowSpawnPt to advSpawnPt >= 8  # Trailing behind adv
require 5 <= (distance from egoSpawnPt to intersection) <= 15  # Near stop line
require 5 <= (distance from advSpawnPt to intersection) <= 15  # Near stop line