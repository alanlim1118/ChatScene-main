description = "Ego vehicle detects a stationary vehicle with lateral offset on a curved road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(15, 25)
param OPT_OFFSET = Range(0.8, 1.5) * Uniform(-1, 1)

curvedLanes = []
for lane in network.lanes:
    if abs(lane.centerline.start.heading - lane.centerline.end.heading) > 0.2:
        curvedLanes.append(lane)

egoLane = Uniform(*curvedLanes)
egoSpawnPt = new OrientedPoint on egoLane.centerline

targetPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_DIST
advSpawnPt = new OrientedPoint left of targetPt by globalParameters.OPT_OFFSET

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE_THRESHOLD = 12

behavior EgoBehavior(target_speed, brake_threshold):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, brake_threshold):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_BRAKE_THRESHOLD)

adversary = new Car at advSpawnPt,
    with blueprint MODEL

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_ADV_DIST + 15)