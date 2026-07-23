description = "Ego vehicle emergency brakes to avoid a head-on collision with an overtaking black SUV on a wet mountain road."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetCloudyNoon'

param OPT_SPAWN_OFFSET = Range(20, 40)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
road = egoLane.road
advLane = Uniform(*road.backwardLanes.lanes)

advLanePt = advLane.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advLanePt for globalParameters.OPT_SPAWN_OFFSET * -1

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

truck = new Truck at advSpawnPt,
	with behavior TruckBehavior()

param OPT_ADV2_SPEED = Range(10, 14)
param OVERTAKE_DIST = 20

behavior OvertakeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV2_SPEED) until withinDistanceToAnyCars(self, globalParameters.OVERTAKE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLane.sections[0], is_oppositeTraffic=True, target_speed=globalParameters.OPT_ADV2_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV2_SPEED)

adversary2 = new Car behind truck by Range(25, 35),
    facing roadDirection,
    with blueprint MODEL,
    with behavior OvertakeBehavior()

require 15 <= (distance from ego to truck) <= 50
require 45 <= (distance from ego to adversary2) <= 90
terminate when (ego.speed < 0.1) and ((distance from ego to egoSpawnPt) > 1)