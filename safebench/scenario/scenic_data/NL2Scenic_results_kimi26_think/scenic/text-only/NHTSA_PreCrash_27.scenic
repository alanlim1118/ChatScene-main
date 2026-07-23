"""Scenario Description:

Vehicle stops at a stop sign in a rural area, in daylight, under clear weather conditions, at an intersection with a posted speed limit of 35 mph; and proceeds to turn left against lateral crossing traffic.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

# Posted speed limit ~35 mph (≈ 15.6 m/s)
param EGO_SPEED = VerifaiRange(13, 15)
param EGO_BRAKE = 1.0

param ADV_SPEED = VerifaiRange(10, 15)

EGO_INIT_DIST = [15, 30]
ADV_INIT_DIST = [10, 30]
STOP_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    # Approach the stop sign
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory) \
        until (distance to intersection) <= STOP_DIST
    # Stop at the stop sign
    take SetBrakeAction(EGO_BRAKE)
    wait 2
    # Proceed to turn left against lateral crossing traffic
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is3Way or i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST