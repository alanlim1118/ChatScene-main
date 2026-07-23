description = "Ego vehicle enters an urban multi-lane roundabout in dark, low-visibility conditions as a vehicle from the left simultaneously enters and abruptly changes lanes into its path."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(2, 4)
param ADV_DIST = Range(15, 25)

behavior AdversaryBehavior():
	rightLaneSec = self.laneSection.laneToRight
	do LaneChangeBehavior(
		laneSectionToSwitch=rightLaneSec,
		target_speed=globalParameters.ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car following roadDirection for globalParameters.ADV_DIST,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param ADV2_SPEED = Range(3, 6)
param ADV2_DIST = Range(30, 45)

adversary_2 = new Car following roadDirection for globalParameters.ADV2_DIST,
	with blueprint MODEL,
	with behavior FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

require 10 <= (distance from ego to adversary) <= 30
require 25 <= (distance from ego to adversary_2) <= 55
terminate when (distance from ego to adversary) < 4