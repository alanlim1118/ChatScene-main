description = "Rear-end collision: A vehicle fails to stop for a stopped vehicle at a red light."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE = Range(10, 15)

# Select a signalized intersection with incoming lanes to represent an urban straight road approach
intersections = filter(lambda i: i.isSignalized and len(i.incomingLanes) > 0, network.intersections)
targetIntersection = Uniform(*intersections)
egoInitLane = Uniform(*targetIntersection.incomingLanes)

# Position the adversarial vehicle on the centerline of the approach lane
advSpawnPt = new OrientedPoint in egoInitLane.centerline

# Position the ego vehicle behind the adversarial vehicle in the same lane
egoSpawnPt = new OrientedPoint behind advSpawnPt by globalParameters.OPT_DISTANCE

param EGO_SPEED = Range(12, 18)
param BRAKE_THRESHOLD = Range(6, 10)
param BRAKE_FORCE = Range(0.4, 0.7)

behavior EgoBehavior(speed, threshold, brake):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED,
        globalParameters.BRAKE_THRESHOLD,
        globalParameters.BRAKE_FORCE
    )

behavior AdversaryBehavior():
    while True:
        wait

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(adversary, 30):
            setClosestTrafficLightStatus(adversary, 'red')
        wait

require monitor TrafficLightMonitor()
require (distance from advSpawnPt to targetIntersection) <= 20
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 15

terminate when (distance from ego to egoSpawnPt) > 50