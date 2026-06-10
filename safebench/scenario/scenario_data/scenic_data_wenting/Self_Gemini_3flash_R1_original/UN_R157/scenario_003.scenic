description = "An adversary vehicle suddenly decelerates in front of the ego vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

forwardSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section.isForward:
            forwardSections.append(section)

egoSection = Uniform(*forwardSections)
egoSpawnPt = new OrientedPoint in egoSection.centerline
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(15, 25)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE = Range(0.7, 1.0)
param BRAKE_THRESHOLD = Range(12, 15)

behavior AdversaryBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.BRAKE_THRESHOLD:
        take SetBrakeAction(globalParameters.ADV_BRAKE)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLights()
require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate after 20 seconds