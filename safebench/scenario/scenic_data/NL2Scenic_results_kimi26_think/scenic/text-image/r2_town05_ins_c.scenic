"""Scenario Description:

A red ego vehicle is traveling in the lower lane of a multi-lane urban road, while a blue adversarial vehicle occupies the adjacent upper lane slightly ahead, moving in the same direction. A light blue trajectory line extends forward from the blue vehicle, indicating its projected path along the road. The street is flanked by a sidewalk and a modern building complex with landscaping and trees on the upper side, and a large green park area populated with numerous trees on the lower side. Intersections with marked crosswalks are visible at the left and right boundaries of the scene, and street lamps line the sidewalks.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_ADV_AHEAD_DIST = Range(5, 15)

OPT_LANE_WIDTH = 3.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior DriveBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight maneuver into an intersection to anchor the scenario
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane

# Ensure there is an adjacent lane for the adversarial vehicle
require egoInitLane.leftLane is not None or egoInitLane.rightLane is not None

# Randomly choose an adjacent lane (left or right)
adjacentLanes = []
if egoInitLane.leftLane is not None:
    adjacentLanes.append(egoInitLane.leftLane)
if egoInitLane.rightLane is not None:
    adjacentLanes.append(egoInitLane.rightLane)
advLane = Uniform(*adjacentLanes)

# Spawn point for the ego vehicle
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Compute a point ahead of the ego in its lane, then shift laterally into the adjacent lane
aheadPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_ADV_AHEAD_DIST

if advLane is egoInitLane.leftLane:
    advSpawnPt = new OrientedPoint left of aheadPt by OPT_LANE_WIDTH,
        with heading aheadPt.heading
else:
    advSpawnPt = new OrientedPoint right of aheadPt by OPT_LANE_WIDTH,
        with heading aheadPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior DriveBehavior(globalParameters.OPT_EGO_SPEED)

AdvAgent = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior DriveBehavior(globalParameters.OPT_ADV_SPEED)

# Ensure the ego is a reasonable distance from the intersection (boundaries)
require 40 <= (distance from ego to intersection) <= 80
# Ensure the adversary is ahead of the ego (closer to the intersection)
require (distance from AdvAgent to intersection) < (distance from ego to intersection)
# Terminate if the ego passes the intersection or drives too far
terminate when distance from ego to intersection > 100