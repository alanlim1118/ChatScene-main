description = "Ego vehicle collides with a white sedan aborting an overtake and cutting back into its lane during a rainstorm on a wet rural road."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'HardRainNoon'

twoLaneRoad = Uniform(*filter(lambda r: r.forwardLanes is not None and r.backwardLanes is not None, network.roads))
egoLane = Uniform(*twoLaneRoad.forwardLanes.lanes)
oppLane = Uniform(*twoLaneRoad.backwardLanes.lanes)

egoSpawnPt = new OrientedPoint in egoLane.centerline
silverSedanSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt.position for Range(20, 40)

oppLanePt = oppLane.centerline.project(egoSpawnPt.position)
whiteSedanSpawnPt = new OrientedPoint following oppLane.orientation from oppLanePt for Range(5, 15)
redCarSpawnPt = new OrientedPoint following oppLane.orientation from whiteSedanSpawnPt for Range(30, 50)
blackSUVSpawnPt = new OrientedPoint following oppLane.orientation from redCarSpawnPt for Range(30, 50)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=10)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at silverSedanSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

param WHITE_ADV_SPEED = Range(8, 12)
param OVERTAKE_DURATION = Range(3, 5)
param WRONG_WAY_DURATION = Range(3, 5)

behavior WhiteSedanBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.WHITE_ADV_SPEED) for globalParameters.OVERTAKE_DURATION seconds
    do FollowLaneBehavior(target_speed=globalParameters.WHITE_ADV_SPEED, laneToFollow=egoLane) for globalParameters.WRONG_WAY_DURATION seconds
    do FollowLaneBehavior(target_speed=globalParameters.WHITE_ADV_SPEED, laneToFollow=oppLane)

whiteSedan = new Car at whiteSedanSpawnPt,
    with blueprint MODEL,
    with behavior WhiteSedanBehavior()

param RED_ADV_SPEED = Range(8, 12)

behavior RedCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.RED_ADV_SPEED)

redCar = new Car at redCarSpawnPt,
    with behavior RedCarBehavior()

param BLACK_SUV_SPEED = Range(8, 12)

behavior BlackSUVBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.BLACK_SUV_SPEED)

blackSUV = new Car at blackSUVSpawnPt,
    with blueprint MODEL,
    with behavior BlackSUVBehavior()

require 20 <= (distance from ego to adversary) <= 40
require 25 <= (distance from whiteSedan to adversary) <= 55
terminate when (whiteSedan intersects ego)