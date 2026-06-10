'''The lead vehicle performs an emergency lane change to avoid a stopped car when its own Time-to-Collision with that car is only 1.5 seconds, creating a high-urgency "late reveal" situation for the ego vehicle following behind.'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    # Follow the lane at the speed defined by OPT_ADV_SPEED until a condition is met
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (
        distance to some_target < globalParameters.OPT_ADV_DISTANCE)
    
    # Perform a lane change to an adjacent lane
    targetLaneSec = network.laneSectionAt(self).adjacentLanes[0]
    do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)

    # Wait for a number of steps directly specified by OPT_WAIT_STEPS
    for _ in range(globalParameters.OPT_WAIT_STEPS):
        wait

    # Stop the vehicle after waiting
    do SetSpeedAction(0)

param OPT_ADV_SPEED = Range(5, 10)  # Speed range for the vehicle
param OPT_ADV_DISTANCE = Range(0, 20)  # Distance threshold for stopping the follow behavior
param OPT_WAIT_STEPS = Range(0, 20)  # Directly used range for wait steps before stopping
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
AdvAgent = Car at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()