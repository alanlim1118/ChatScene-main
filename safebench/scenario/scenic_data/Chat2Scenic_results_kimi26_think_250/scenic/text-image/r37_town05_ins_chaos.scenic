description = "Ego vehicle executes a left turn at a busy urban intersection with a leading right-turning vehicle, cross traffic, and oncoming vehicles."
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

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(trajectory=advTrajectory)

behavior ForwardAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

forward_adv = new Car ahead of advSpawnPt by 30,
    facing advSpawnPt.heading,
    with blueprint MODEL,
    with behavior ForwardAdvBehavior()

behavior TravelForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv_forward = new Car ahead of advSpawnPt by Range(20, 40),
	facing advSpawnPt.heading,
	with blueprint MODEL,
	with behavior TravelForwardBehavior()

behavior ForwardDrivingBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv_straight = new Car behind advSpawnPt by Range(10, 20),
	facing advSpawnPt.heading,
	with behavior ForwardDrivingBehavior()

behavior CarForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv_forward_car = new Car ahead of advSpawnPt by Range(50, 60),
	facing advSpawnPt.heading,
	with behavior CarForwardBehavior()

behavior MoveForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv_front = new Car ahead of advSpawnPt by Range(40, 50),
	facing advSpawnPt.heading,
	with behavior MoveForwardBehavior()

behavior StraightForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv_cruising = new Car behind advSpawnPt by Range(25, 35),
	facing advSpawnPt.heading,
	with behavior StraightForwardBehavior()

EGO_TO_INTERSECTION = [15, 35]
EGO_TO_LEAD = [10, 30]
TERM_DIST = 100

require EGO_TO_INTERSECTION[0] <= (distance to intersection) <= EGO_TO_INTERSECTION[1]
require EGO_TO_LEAD[0] <= (distance from ego to adversary) <= EGO_TO_LEAD[1]
terminate when (distance to egoSpawnPt) > TERM_DIST