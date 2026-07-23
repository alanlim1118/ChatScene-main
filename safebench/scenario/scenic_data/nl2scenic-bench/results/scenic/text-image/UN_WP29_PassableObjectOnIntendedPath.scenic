"""Scenario Description:

In a top-down view of a traffic scenario, a green ego vehicle is driving straight in a lane bounded by a dashed white line above and a solid yellow line below. A green arrow indicates the vehicle's forward motion towards a green rectangular object located directly ahead in the same lane. This object is described as a passable item, such as a manhole lid or a small branch, which the ego vehicle must react to while pursuing its objective of continuing straight along the road.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
OBSTACLE_MODEL = 'static.prop.manholecover'

param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)
param REACTION_DIST = VerifaiRange(15, 25)
PASS_DIST = 3
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when (distance from self to obstacle) < globalParameters.REACTION_DIST:
        take SetThrottleAction(0)
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when (distance from self to obstacle) < globalParameters.PASS_DIST:
        take SetThrottleAction(0.5)
        take SetBrakeAction(0)
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED * 0.5, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment with appropriate lane markings
egoLane = Uniform(*filter(lambda l: l.maneuvers and any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), network.lanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoLane.maneuvers))
egoTrajectory = [egoLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Spawn ego on the lane centerline
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Place obstacle ahead of ego in the same lane
obstacleOffset = Range(30, 50)
obstacleSpawnPt = new OrientedPoint at egoSpawnPt.offsetBy(dx=obstacleOffset),
    facing egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 255, 0),
    with behavior EgoBehavior(egoTrajectory)

obstacle = new Object at obstacleSpawnPt,
    with blueprint OBSTACLE_MODEL,
    with color (0, 200, 0),
    with width 1.0,
    with length 1.0,
    with height 0.1

require obstacle in egoLane
require 30 <= (distance from ego to obstacle) <= 50
terminate when (distance from ego to egoSpawnPt) > TERM_DIST