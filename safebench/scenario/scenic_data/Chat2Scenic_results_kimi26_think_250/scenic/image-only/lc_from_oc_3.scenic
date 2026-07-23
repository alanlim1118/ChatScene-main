description = "Ego vehicle changes lanes from oncoming traffic into a gap between two vehicles in the adjacent lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(7, 10)
LANE_CHANGE_TIME = 5

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for LANE_CHANGE_TIME seconds
	do LaneChangeBehavior(laneSectionToSwitch=self.laneSection.fasterLane, target_speed=globalParameters.EGO_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param ADV2_SPEED = Range(7, 10)

behavior AdvForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdvForwardBehavior()

param ADV3_SPEED = Range(7, 10)

behavior Adv3ForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV3_SPEED)

adversary3 = new Car at advSpawnPt,
	with behavior Adv3ForwardBehavior()