"""Scenario Description:

In a simulated top-down view of a four-way intersection, a white ego vehicle travels horizontally from left to right along the upper lane of the main road, maintaining a steady speed. Parallel to it in the lower lane, a long red vehicle moves in the same direction. A motorcyclist, visible as a small black object, enters the junction from the bottom vertical road and proceeds upwards on a perpendicular path, crossing directly into the ego vehicle's trajectory. The ego vehicle continues straight without braking or steering away, resulting in a collision where the frontal structure of the white vehicle strikes the front of the motorcyclist as they converge in the center of the intersection.

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
RED_VEHICLE_MODEL = "vehicle.carlamotors.carlacola"
MOTORCYCLE_MODEL = "vehicle.kawasaki.ninja"

param EGO_SPEED = Range(5, 10)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: straight maneuver on the main road
allStraight = list(filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoManeuver = Uniform(*allStraight)
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Motorcyclist: straight maneuver from the crossing road
reverseManeuvers = list(egoManeuver.reverseManeuvers)
crossStraight = [m for m in allStraight if m is not egoManeuver and m not in reverseManeuvers]
require len(crossStraight) > 0
motorcycleManeuver = Uniform(*crossStraight)
motorcycleInitLane = motorcycleManeuver.startLane
motorcycleTrajectory = [motorcycleInitLane, motorcycleManeuver.connectingLane, motorcycleManeuver.endLane]

# Spawn points along the selected lanes
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
motorcycleSpawnPt = new OrientedPoint in motorcycleInitLane.centerline

# Red vehicle: parallel lane in the same direction as the ego
laneSection = network.laneSectionAt(egoSpawnPt)
require laneSection is not None
require laneSection.laneToRight is not None
redLane = laneSection.laneToRight.lane
require abs(redLane.centerline.start.heading - egoInitLane.centerline.start.heading) < 10 deg
redSpawnPt = new OrientedPoint in redLane.centerline

# Compute motorcycle speed so both agents reach the intersection at the same time
egoDist = distance from egoSpawnPt to intersection
motoDist = distance from motorcycleSpawnPt to intersection
require egoDist > 0
require motoDist > 0
motoSpeed = globalParameters.EGO_SPEED * motoDist / egoDist

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color White,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

redVehicle = new Car at redSpawnPt,
    with regionContainedIn None,
    with blueprint RED_VEHICLE_MODEL,
    with color Red,
    with behavior FollowLaneBehavior(globalParameters.EGO_SPEED)

motorcyclist = new Motorcycle at motorcycleSpawnPt,
    with regionContainedIn None,
    with blueprint MOTORCYCLE_MODEL,
    with color Black,
    with behavior FollowTrajectoryBehavior(target_speed=motoSpeed, trajectory=motorcycleTrajectory)

# Keep vehicles at reasonable distances from the intersection and roughly abreast
require 30 <= (distance from egoSpawnPt to intersection) <= 60
require 30 <= (distance from motorcycleSpawnPt to intersection) <= 60
require abs((distance from redSpawnPt to intersection) - (distance from egoSpawnPt to intersection)) < 5

terminate when (distance to egoSpawnPt) > 80