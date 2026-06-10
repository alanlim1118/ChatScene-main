'''While performing a maneuver, the ego-vehicle encounters an obstacle in the road, either a pedestrian or a bicycle, and must perform an emergency brake or an avoidance maneuver.'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # The adversarial vehicle waits until it is within a significant range to impact the ego's maneuver.
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_BRAKE_DISTANCE
    # Once within a critical distance of the ego's trajectory, the adversarial vehicle initiates a sudden braking maneuver.
    while True:
        take SetBrakeAction(globalParameters.OPT_BRAKE)  # Continues to apply brakes to simulate an emergency or unexpected stop.

param OPT_ADV_SPEED = Range(5, 15)  # The speed at which the adversarial vehicle approaches the intersection.
param OPT_BRAKE_DISTANCE = Range(0, 4)  # The critical distance at which the adversarial vehicle begins its sudden stop.
param OPT_BRAKE = Range(0, 1)  # The intensity of the braking action, potentially simulating an abrupt or gentle stop.
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Pedestrian or Bicycle at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()