'''The ego vehicle travels in a straight line for at least two seconds before encountering a stationary adult pedestrian target positioned directly in its driving path and facing away from the vehicle, requiring the ego vehicle to detect the target and execute an autonomous braking maneuver to avoid a collision'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    initialDirection = self.heading
    while (distance to self) > globalParameters.OPT_ADV_DISTANCE:
        wait
    while True:
        take SetWalkingDirectionAction(initialDirection)
        take SetWalkingSpeedAction(globalParameters.OPT_ADV_SPEED)
        for _ in range(globalParameters.OPT_WAIT_STEP_1):
            wait
        take SetWalkingSpeedAction(0)  # Stop suddenly
        for _ in range(globalParameters.OPT_WAIT_STEP_2):
            wait

param OPT_ADV_SPEED = Range(0, 5)
param OPT_ADV_DISTANCE = Range(0, 10)
param OPT_WAIT_STEP_1 = Range(0, 30)  # Wait time in steps for the first direction
param OPT_WAIT_STEP_2 = Range(0, 30)  # Wait time in steps for the second direction
intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
param OPT_EGO_SPEED = 10

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoManeuver.startLane.centerline

# Setting up the ego vehicle at the initial position
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
require 10 <= (distance to intersection) <= 40
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Pedestrian at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()