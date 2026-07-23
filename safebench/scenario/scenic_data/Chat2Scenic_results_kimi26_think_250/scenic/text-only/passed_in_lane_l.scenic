description = "Ego vehicle drives straight within its lane while an adversarial motorcycle passes from the adjacent left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward == laneSec.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToLeft
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(target_speed):
	do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_PASS_SPEED = Range(11, 15)

behavior MotorcyclePassBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_PASS_SPEED)

adv = new Motorcycle at advSpawnPt,
    with behavior MotorcyclePassBehavior()