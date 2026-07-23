description = "Blue ego vehicle in lower lane approaches a pink adversarial vehicle reversing leftward into the upper lane."
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

param ADV_DIST = Range(20, 30)
param ADV_THROTTLE = Range(0.3, 0.5)

behavior AdversaryBehavior():
    take SetReverseAction(True)
    while True:
        take RegulatedControlAction(globalParameters.ADV_THROTTLE, -0.7, -0.7)

adversary = new Car ahead of egoSpawnPt by globalParameters.ADV_DIST,
    with blueprint MODEL,
    with behavior AdversaryBehavior()