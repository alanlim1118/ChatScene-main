description = "Ego vehicle follows a front vehicle through a busy urban four-way intersection with an adversary turning left from the left arm and two vehicles turning left and right from the right arm under dark weather conditions."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(7, 10)
behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_STRAIGHT_SPEED = Range(7, 10)

behavior AdvStraightBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_STRAIGHT_SPEED)

advStraight = new Car ahead of egoSpawnPt by Range(20, 30),
	facing egoSpawnPt.heading,
	with blueprint MODEL,
	with behavior AdvStraightBehavior()

param ADV_LEFT_TURN_SPEED = Range(7, 10)

behavior AdvLeftTurnBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_LEFT_TURN_SPEED, trajectory=trajectory)

advLeftTurn = new Car left of egoSpawnPt by Range(20, 30),
	facing toward egoSpawnPt,
	with blueprint MODEL,
	with behavior AdvLeftTurnBehavior(advLeftTurnTraj)

param ADV_RIGHT_TURN_SPEED = Range(7, 10)

behavior AdvRightTurnBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_RIGHT_TURN_SPEED, trajectory=trajectory)

advRightTurn = new Car right of egoSpawnPt by Range(20, 30),
	facing toward egoSpawnPt,
	with blueprint MODEL,
	with behavior AdvRightTurnBehavior(advRightTurnTraj)

require 20 <= (distance from ego to advStraight) <= 30
require 20 <= (distance from ego to advLeftTurn) <= 30
require 20 <= (distance from ego to advRightTurn) <= 30
require (distance to intersection) <= 60
terminate when distance from ego to intersection > 60