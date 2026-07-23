description = "Ego vehicle collides with a white sedan aborting an overtake and cutting back into its lane during a rainstorm on a wet rural road."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'HardRainNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
twoLaneRoad = egoLane.road
oppLane = Uniform(*twoLaneRoad.backwardLanes.lanes)

silverSedanSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt.position for Range(20, 40)

oppLanePt = oppLane.centerline.project(egoSpawnPt.position)
whiteSedanSpawnPt = new OrientedPoint following oppLane.orientation from oppLanePt for Range(5, 15)
redCarSpawnPt = new OrientedPoint following oppLane.orientation from whiteSedanSpawnPt for Range(30, 50)
blackSUVSpawnPt = new OrientedPoint following oppLane.orientation from redCarSpawnPt for Range(30, 50)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at silverSedanSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

param OPT_WHITE_ADV_SPEED = Range(8, 12)
param OPT_OVERTAKE_DURATION = Range(3, 5)
param OPT_WRONG_WAY_DURATION = Range(3, 5)

behavior WhiteSedanBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_ADV_SPEED) for globalParameters.OPT_OVERTAKE_DURATION seconds
    do FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_ADV_SPEED, laneToFollow=egoLane) for globalParameters.OPT_WRONG_WAY_DURATION seconds
    do FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_ADV_SPEED, laneToFollow=oppLane)

whiteSedan = new Car at whiteSedanSpawnPt,
    with blueprint MODEL,
    with behavior WhiteSedanBehavior()

param OPT_RED_ADV_SPEED = Range(8, 12)

behavior RedCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_RED_ADV_SPEED)

redCar = new Car at redCarSpawnPt,
    with behavior RedCarBehavior()

param OPT_BLACK_SUV_SPEED = Range(8, 12)

behavior BlackSUVBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_BLACK_SUV_SPEED)

blackSUV = new Car at blackSUVSpawnPt,
    with blueprint MODEL,
    with behavior BlackSUVBehavior()

require 20 <= (distance from ego to adversary) <= 40
require 25 <= (distance from whiteSedan to adversary) <= 55
terminate when (whiteSedan intersects ego)