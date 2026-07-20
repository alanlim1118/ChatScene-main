'''Vehicle A was braking for stopped traffic. Driver B, coming from some distance behind A, saw the brake lights. When B braked the road was very slick. B did not stop and struck A in the rear.'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_ADV_SPEED = Range(5, 10)
param OPT_ADV_DISTANCE = Range(0, 20)
param OPT_BRAKE = Range(0, 1)
param OPT_PAUSE_DURATION = Range(10, 30)
param OPT_THROTTLE = Range(0.5, 1)  # Throttle intensity for acceleration

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance to self) < globalParameters.OPT_ADV_DISTANCE
    while True:
        take SetBrakeAction(globalParameters.OPT_BRAKE)
        for _ in range(globalParameters.OPT_PAUSE_DURATION):
            wait
        take SetThrottleAction(globalParameters.OPT_THROTTLE)

## MONITORS
monitor TrafficLights:
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent, 100):
            setClosestTrafficLightStatus(AdvAgent, "red")
        wait

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Setting up the ego vehicle at the initial position
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL

require 10 <= (distance to intersection) <= 40
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