'''Vehicle is going straight in an urban area, in daylight, under clear weather conditions, at an intersection-related location with a posted speed limit of 45 mph; and closes in on an accelerating lead vehicle'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # The adversarial car waits until it is within 60 meters of the ego vehicle.
    do FollowLaneBehavior(globalParameters.OPT_ADV_SPEED) until (distance to self) < globalParameters.OPT_ADV_DISTANCE
    while True:
        take SetThrottleAction(globalParameters.OPT_ADV_THROTTLE)  # Aggressively adjusts its speed to close the gap.

param OPT_ADV_SPEED = Range(0, 10)  # Controls the initial speed of the adversarial car.
param OPT_ADV_DISTANCE = Range(0, 20)  # Specifies the distance at which the car begins its aggressive maneuver.
param OPT_ADV_THROTTLE = Range(0.5, 1)  # Determines the intensity of the acceleration.
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
param OPT_EGO_SPEED = 10

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Car at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()