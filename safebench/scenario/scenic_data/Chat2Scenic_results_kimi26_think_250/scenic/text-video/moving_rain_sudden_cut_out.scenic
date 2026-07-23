description = "Ego vehicle collides with a lead SUV after emergency braking and evasive swerving on a wet, rainy highway."\nparam map = localPath('../../maps/Town04.xodr')\nparam carla_map = 'Town04'\nmodel scenic.simulators.carla.model\nMODEL = 'vehicle.lincoln.mkz_2017'\nparam weather = 'MidRainyNoon'

initLane = Uniform(*filter(lambda lane:
	all([sec._laneToLeft is not None and sec._laneToLeft.isForward is not sec.isForward for sec in lane.sections]),
	network.lanes))

egoSpawnPt = new OrientedPoint in initLane.centerline

carSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(20, 30)

obstacleSpawnPt = new OrientedPoint following roadDirection from carSpawnPt for Range(40, 60)

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE = Range(0.4, 0.7)
param ADV_STEER = Range(0.5, 0.8)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for Range(3, 5) seconds
    past_steer = 0
    while True:
        take RegulatedControlAction(throttle=-globalParameters.ADV_BRAKE, steer=globalParameters.ADV_STEER, past_steer=past_steer, max_throttle=0.5, max_brake=0.8, max_steer=0.8)
        past_steer = globalParameters.ADV_STEER

adversary = new Car at carSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

terminate when ego intersects adversary