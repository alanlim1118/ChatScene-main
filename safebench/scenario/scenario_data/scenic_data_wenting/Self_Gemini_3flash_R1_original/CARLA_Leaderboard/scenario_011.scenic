description = "Ego vehicle maneuvers to allow an emergency vehicle to pass from behind."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
advSpawnPt = new OrientedPoint behind egoSpawnPt by Range(15, 25)

param EGO_SPEED = Range(7, 10)
param DETECTION_DIST = Range(25, 30)

behavior EgoBehavior(speed, dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyCars(self, dist):
        # Attempt to identify an adjacent lane to move into
        target_sec = None
        # Use the initial lane's section to find neighboring lanes
        if egoInitLane.sections[0]._laneToRight:
            target_sec = egoInitLane.sections[0]._laneToRight
        elif egoInitLane.sections[0]._laneToLeft:
            target_sec = egoInitLane.sections[0]._laneToLeft
        
        if target_sec:
            do LaneChangeBehavior(laneSectionToSwitch=target_sec, target_speed=speed)
        
        # Slow down to allow the emergency vehicle to pass from behind
        do FollowLaneBehavior(target_speed=speed * 0.5) for 10 seconds
        terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.DETECTION_DIST)

param OPT_ADV_SPEED = Range(12, 15)

behavior AdvBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

advVehicle = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advVehicle, 100):
            setClosestTrafficLightStatus(advVehicle, "green")
        wait

require monitor TrafficLights()
require 15 <= (distance from advSpawnPt to egoSpawnPt) <= 25

terminate when (distance from advVehicle to egoSpawnPt) > 100
terminate after 60 seconds