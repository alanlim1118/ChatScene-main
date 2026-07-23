description = "Ego vehicle follows a red lead vehicle through a left turn at a signalized urban intersection while navigating conflicting straight-moving traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(5, 8)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car left of (ahead of egoSpawnPt by Range(20, 30)) by Range(3, 5),
	facing 180 deg relative to egoSpawnPt.heading,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param ADV2_SPEED = Range(8, 12)

behavior Adversary2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car right of (ahead of egoSpawnPt by Range(20, 30)) by Range(10, 15),
	facing -90 deg relative to egoSpawnPt.heading,
	with blueprint MODEL,
	with behavior Adversary2Behavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        wait

require monitor TrafficLights()
require 18 <= (distance from ego to adversary) <= 32
require 40 <= (distance from ego to intersection) <= 60
terminate when (distance from ego to intersection) > 60