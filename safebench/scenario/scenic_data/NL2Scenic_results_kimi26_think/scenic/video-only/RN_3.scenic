"""Scenario Description:

Approaching a roundabout, the ego vehicle (green box) is initially positioned in the left lane but intends to execute a right turn, realizing it is in the incorrect lane for this maneuver. Consequently, the ego vehicle performs a lane change into the right lane, directly cutting off an adversary vehicle (yellow box) that is already traveling in that lane. This improper merge forces the ego vehicle into the path of the yellow vehicle as they both near the roundabout entry, creating a conflict where the ego vehicle asserts its position in the right lane to facilitate the turn.
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

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [15, 25]
ADV_INIT_DIST = [15, 25]

param EGO_SPEED = VerifaiRange(8, 12)
param ADV_SPEED = VerifaiRange(6, 10)
param LANE_CHANGE_DIST = VerifaiRange(15, 25)

CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(init_lane, target_lane, right_turn_trajectory, target_speed):
    try:
        # Travel in the initial (left) lane approaching the roundabout
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=[init_lane]) \
            until (distance to intersection) <= globalParameters.LANE_CHANGE_DIST
        
        # Cut across into the right lane (target_lane) in front of the adversary
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=[target_lane]) \
            until (distance to intersection) <= 8
        
        # Enter the roundabout and execute the right turn
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=right_turn_trajectory)
    
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*network.intersections)

# Select an incoming lane for the ego (treated as the left lane)
egoInitLane = Uniform(*intersection.incomingLanes)

# Select another incoming lane on the same road (treated as the right lane)
sameRoadLanes = list(filter(lambda l: l.road is egoInitLane.road, intersection.incomingLanes))
require len(sameRoadLanes) >= 2
advInitLane = Uniform(*filter(lambda l: l is not egoInitLane, sameRoadLanes))

# Ego intends to turn right at the roundabout, so it must end up in the right lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, advInitLane.maneuvers))
egoTrajectory = [advInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary continues through the roundabout (straight/through maneuver)
advManeuver = Uniform(*filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.RIGHT_TURN), advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoInitLane, advInitLane, egoTrajectory, globalParameters.EGO_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Ensure both vehicles are approaching the intersection and near each other
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require abs((distance to intersection) - (distance from adversary to intersection)) <= 12

terminate when (distance to egoSpawnPt) > TERM_DIST
terminate when (distance from adversary to advSpawnPt) > TERM_DIST