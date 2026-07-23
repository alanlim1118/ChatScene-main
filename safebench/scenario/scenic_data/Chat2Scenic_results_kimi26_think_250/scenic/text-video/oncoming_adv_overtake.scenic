description = "Ego vehicle emergency brakes to avoid a head-on collision with an overtaking black SUV on a wet mountain road."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetCloudyNoon'

param OPT_SPAWN_OFFSET = Range(20, 40)

twoLaneRoads = filter(lambda r: r.forwardLanes is not None and r.backwardLanes is not None and len(r.forwardLanes.lanes) > 0 and len(r.backwardLanes.lanes) > 0, network.roads)
road = Uniform(*twoLaneRoads)

egoLane = Uniform(*road.forwardLanes.lanes)
advLane = Uniform(*road.backwardLanes.lanes)

egoSpawnPt = new OrientedPoint in egoLane.centerline

advLanePt = advLane.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advLanePt for globalParameters.OPT_SPAWN_OFFSET * -1

param EGO_SPEED = Range(7, 10)
EGO_BRAKE = 1.0
SAFE_DIST = 10

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, SAFE_DIST):
        take SetBrakeAction(EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

truck = new Truck at advSpawnPt,
	with behavior TruckBehavior()

param ADV2_SPEED = Range(10, 14)
param OVERTAKE_DIST = 20

behavior OvertakeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED) until withinDistanceToAnyCars(self, globalParameters.OVERTAKE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLane.sections[0], is_oppositeTraffic=True, target_speed=globalParameters.ADV2_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car behind truck by Range(25, 35),
    facing roadDirection,
    with blueprint MODEL,
    with behavior OvertakeBehavior()

require 15 <= (distance from ego to truck) <= 50
require 45 <= (distance from ego to adversary2) <= 90
terminate when (ego.speed < 0.1) and ((distance from ego to egoSpawnPt) > 1)