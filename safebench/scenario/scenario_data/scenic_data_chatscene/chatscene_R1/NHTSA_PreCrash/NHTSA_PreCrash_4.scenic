'''Vehicle is going straight in a rural area, in daylight, under clear weather conditions, with a posted speed limit of 35 mph or less; and runs a s sign at an intersection'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # Wait until the vehicle is close enough to influence the ego's path.
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_ADV_DISTANCE
    # Executes a left turn maneuver.
    take SetSteerAction(globalParameters.OPT_STEER)  # Full left turn
    while True:
        take SetSpeedAction(0)  # Eventually stops after completing the turn

param OPT_ADV_SPEED = Range(5, 15)  # Speed at which the vehicle approaches the intersection.
param OPT_ADV_DISTANCE = Range(0, 5)  # The critical distance to start the turn.
param OPT_STEER = Range(-1.0, 0.0)
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
param OPT_GEO_BLOCKER_X_DISTANCE = Range(-8, -2)  # Negative range for left side
param OPT_GEO_BLOCKER_Y_DISTANCE = Range(15, 50)
param OPT_GEO_X_DISTANCE = Range(-2, 2)
param OPT_GEO_Y_DISTANCE = Range(2, 6)

LeftFrontSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
Blocker = Car left of LeftFrontSpawnPt by globalParameters.OPT_GEO_BLOCKER_X_DISTANCE,
    with heading LeftFrontSpawnPt.heading,
    with regionContainedIn None

SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Car at Blocker offset along LeftFrontSpawnPt.heading by SHIFT,
    with heading LeftFrontSpawnPt.heading - 90 deg,  # Adjusted for spawning from the left
    with regionContainedIn None,
    with behavior AdvBehavior()