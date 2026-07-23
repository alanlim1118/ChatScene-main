description = "Ego vehicle follows highway traffic while an adversary merges from a curved on-ramp."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

mergePairs = []
for inter in network.intersections:
    for out_lane in inter.outgoingLanes:
        if out_lane.sections[0]._laneToLeft is not None and all([sec._laneToRight is None for sec in out_lane.sections]):
            for in_lane in inter.incomingLanes:
                if in_lane is not out_lane and in_lane.sections[0]._laneToLeft is None and in_lane.sections[0]._laneToRight is None:
                    mergePairs.append((out_lane, in_lane))
egoLane, advLane = Uniform(*mergePairs)
egoSpawnPt = new OrientedPoint in egoLane.centerline
advSpawnPt = new OrientedPoint in advLane.centerline
param OPT_FRONT_DIST = Range(20, 40)
param OPT_REAR_DIST = Range(-40, -20)
frontSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_FRONT_DIST
rearSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_REAR_DIST

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

behavior FrontCarBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

frontCar = new Car at frontSpawnPt,
	with blueprint MODEL,
	with behavior FrontCarBehavior()

behavior MergeBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=[advLane, egoLane])

merger = new Car following roadDirection from advSpawnPt for Range(10, 30),
    with blueprint MODEL,
    with behavior MergeBehavior()