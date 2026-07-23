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

param OPT_EGO_SPEED = Range(1, 5)
param OPT_ADV_SPEED = Range(1, 5)
param OPT_GREEN_SPEED = Range(1, 5)
param OPT_YELLOW_SPEED = Range(1, 5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

behavior BlueAdvBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, blueTrajectory)

behavior GreenBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_GREEN_SPEED, greenTrajectory)

behavior YellowBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_YELLOW_SPEED, yellowTrajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: straight from upper lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoRoad = egoInitLane.road

# All maneuvers from the same approach road
sameRoadManeuvers = [m for m in intersection.maneuvers if m.startLane.road is egoRoad]

# Blue adversary: right turn from a different lane (lower lane) on the same road
blueManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.startLane != egoInitLane, sameRoadManeuvers))
blueInitLane = blueManeuver.startLane

# Green car: right turn from the same lane as ego (upper lane)
greenManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.startLane == egoInitLane, sameRoadManeuvers))

# Yellow car: straight from the same lane as blue (lower lane)
yellowManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane == blueInitLane, sameRoadManeuvers))

# Trajectories
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
blueTrajectory = [blueManeuver.startLane, blueManeuver.connectingLane, blueManeuver.endLane]
greenTrajectory = [greenManeuver.startLane, greenManeuver.connectingLane, greenManeuver.endLane]
yellowTrajectory = [yellowManeuver.startLane, yellowManeuver.connectingLane, yellowManeuver.endLane]

# Spawn points on the approach
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
blueSpawnPt = new OrientedPoint in blueInitLane.centerline
greenSpawnPt = new OrientedPoint at egoSpawnPt offset by (-Range(4, 6), 0)
yellowSpawnPt = new OrientedPoint at blueSpawnPt offset by (-Range(4, 6), 0)

# Parking lot to the bottom left of the intersection
lotRef = new OrientedPoint at intersection, facing egoSpawnPt.heading
parkPt1 = new OrientedPoint at lotRef offset by (-20, -8)
parkPt2 = new OrientedPoint at lotRef offset by (-24, -8)
parkPt3 = new OrientedPoint at lotRef offset by (-28, -8)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle: red car in upper lane at the stop line
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (1, 0, 0),
    with behavior EgoBehavior()

# Primary adversarial vehicle: blue car in lower lane, turning right
blueAdv = new Car at blueSpawnPt,
    with heading blueSpawnPt.heading,
    with regionContainedIn None,
    with color (0, 0, 1),
    with behavior BlueAdvBehavior()

# Green car: trailing behind ego in upper lane, turning right
greenCar = new Car at greenSpawnPt,
    with heading greenSpawnPt.heading,
    with regionContainedIn None,
    with color (0, 1, 0),
    with behavior GreenBehavior()

# Yellow car: behind blue in lower lane, going straight across
yellowCar = new Car at yellowSpawnPt,
    with heading yellowSpawnPt.heading,
    with regionContainedIn None,
    with color (1, 1, 0),
    with behavior YellowBehavior()

# Stationary vehicles in the parking lot to the bottom left
greySUV = new Car at parkPt1,
    with heading lotRef.heading + 90 deg,
    with regionContainedIn None,
    with color (0.5, 0.5, 0.5),
    with behavior WaitBehavior()

yellowSUV = new Car at parkPt2,
    with heading lotRef.heading + 90 deg,
    with regionContainedIn None,
    with color (1, 1, 0),
    with behavior WaitBehavior()

redSedan = new Car at parkPt3,
    with heading lotRef.heading + 90 deg,
    with regionContainedIn None,
    with color (1, 0, 0),
    with behavior WaitBehavior()

require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from blueSpawnPt to intersection) <= 40