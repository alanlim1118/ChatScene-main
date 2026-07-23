description = "Ego-vehicle crosses a lane of moving traffic to exit the highway at an off-ramp."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

advInitLane = Uniform(*filter(lambda lane: any([sec._laneToLeft is not None for sec in lane.sections]) and any([m.endLane.road != lane.road for m in lane.maneuvers]), network.lanes))
advLaneSec = Uniform(*filter(lambda sec: sec._laneToLeft is not None, advInitLane.sections))
egoInitLane = advLaneSec._laneToLeft.lane
advSpawnPt = new OrientedPoint in advInitLane.centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

param OPT_EGO_SPEED = Range(10, 15)
param OPT_EGO_LC_DISTANCE = Range(15, 35)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to egoSpawnPt > globalParameters.OPT_EGO_LC_DISTANCE)
    do LaneChangeBehavior(laneSectionToSwitch=advLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(10, 15)

behavior AdvBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
	facing advSpawnPt.heading,
	with behavior AdvBehavior()