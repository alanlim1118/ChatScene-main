description = "Ego vehicle follows lead car while adversarial vehicle merges from right entrance ramp."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(8, 13)

behavior RampMergeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

rampCar = new Car right of egoSpawnPt by 10,
    facing egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior RampMergeBehavior()