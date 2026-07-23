"""Scenario Description:

In this top-down aerial view of a residential T-intersection, a green ego vehicle travels eastbound on the horizontal main road and initiates a right turn into the vertical side street, following a curved green trajectory line. Simultaneously, a blue adversarial vehicle approaches from the west on the opposite side of the main road and executes a left turn into the same vertical street, marked by a yellow trajectory line, creating a conflict as both vehicles attempt to merge into the same lane. A red car follows directly behind the green ego vehicle, while a yellow car trails the blue adversarial vehicle. The scene is framed by a row of townhouses at the top, a large modern building on the bottom left, and a grassy field on the bottom right, with trees lining the sidewalks and a "STOP" marking painted on the road surface at the bottom entrance of the vertical street.

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
ADV_MODEL = "vehicle.lincoln.mkz_2017"
FOLLOWER_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_FOLLOWER_SPEED = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)

behavior FollowerBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(speed, trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 3-way (T) intersection
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego: eastbound right turn into the vertical side street
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary: westbound left turn from the opposite side, merging into the same end lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points on the respective lane centerlines
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Followers spawn behind their respective lead vehicles along the same lane
redSpawnPt = new OrientedPoint at egoSpawnPt offset by -Range(5, 10) @ 0
yellowSpawnPt = new OrientedPoint at advSpawnPt offset by -Range(5, 10) @ 0

#################################
# SCENARIO SPECIFICATION        #
#################################

# Green ego vehicle turning right
ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color [0, 255, 0],
    with behavior EgoBehavior()

# Blue adversarial vehicle turning left
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color [0, 0, 255],
    with behavior AdvBehavior()

# Red car following directly behind the ego vehicle
redCar = new Car at redSpawnPt,
    with heading redSpawnPt.heading,
    with regionContainedIn None,
    with blueprint FOLLOWER_MODEL,
    with color [255, 0, 0],
    with behavior FollowerBehavior(egoTrajectory, globalParameters.OPT_FOLLOWER_SPEED)

# Yellow car trailing the adversarial vehicle
yellowCar = new Car at yellowSpawnPt,
    with heading yellowSpawnPt.heading,
    with regionContainedIn None,
    with blueprint FOLLOWER_MODEL,
    with color [255, 255, 0],
    with behavior FollowerBehavior(advTrajectory, globalParameters.OPT_FOLLOWER_SPEED)

#################################
# CONSTRAINTS                   #
#################################

# Ensure correct approach directions (eastbound vs westbound)
require abs(egoSpawnPt.heading) < 15 deg
require abs(advSpawnPt.heading - 180 deg) < 15 deg

# Ensure both turning vehicles are a reasonable distance from the intersection
require 20 <= (distance from egoSpawnPt to intersection) <= 50
require 20 <= (distance from advSpawnPt to intersection) <= 50

# Ensure followers are behind their respective leaders
require (distance from redSpawnPt to intersection) > (distance from egoSpawnPt to intersection)
require (distance from yellowSpawnPt to intersection) > (distance from advSpawnPt to intersection)

# Ensure both vehicles merge into the same target lane
require advManeuver.endLane is egoManeuver.endLane