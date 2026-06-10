description = "Target vehicle merges into ego vehicle's path with a full lateral displacement."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoLane = Uniform(*filter(lambda l: any(s._laneToLeft or s._laneToRight for s in l.sections), network.lanes))
egoSection = Uniform(*filter(lambda s: s._laneToLeft or s._laneToRight, egoLane.sections))
egoSpawnPt = new OrientedPoint in egoSection.centerline
leftSec = egoSection._laneToLeft
if leftSec is not None:
    adjSection = leftSec
else:
    adjSection = egoSection._laneToRight
advSpawnPt = new OrientedPoint in adjSection.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param OPT_ADV_SPEED = globalParameters.EGO_SPEED - Range(1, 2)
param OPT_ADV_DISTANCE = Range(10, 15)

behavior AdvBehavior(speed, merge_dist):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < merge_dist
    do LaneChangeBehavior(laneSectionToSwitch=egoSection, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

AdvAgent = new Car ahead of egoSpawnPt by Range(20, 30),
    facing egoSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE)

require ego can see AdvAgent
terminate when (distance from ego to egoSpawnPt) > 150
terminate after 60 seconds