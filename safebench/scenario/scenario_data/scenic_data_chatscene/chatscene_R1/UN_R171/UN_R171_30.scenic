'''The VUT travels in a straight line for at least two seconds before encountering a stationary bicycle target positioned directly in its driving path and facing away, requiring the ego vehicle to either execute an emergency braking maneuver or perform a controlled lateral bypass to avoid a collision.'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior WalkStraightBehavior(direction, speed):
    while True:
        take SetWalkingDirectionAction(direction)
        take SetWalkingSpeedAction(speed)

behavior AdvBehavior():
    while (distance to self) > 60:
        wait
    do WalkStraightBehavior(IntSpawnPt.heading + 180 deg, globalParameters.OPT_ADV_SPEED) until (distance to self) < globalParameters.OPT_ADV_DISTANCE
    take SetWalkingSpeedAction(0)

param OPT_ADV_DISTANCE = Range(0, 20)
param OPT_ADV_SPEED = Range(0, 5)
intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoManeuver.startLane.centerline

# Setting up the ego vehicle at the initial position
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL

require 10 <= (distance to intersection) <= 40
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Bicycle at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()