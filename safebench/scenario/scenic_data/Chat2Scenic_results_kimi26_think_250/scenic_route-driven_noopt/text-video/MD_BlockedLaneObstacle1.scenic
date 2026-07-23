description = "Ego vehicle blocked by stopped adversaries at a four-way intersection while adjacent lane traffic flows freely."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
intersection = Uniform(*filter(lambda m: m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers)).intersection
darkAdvSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 20)
darkAdv2SpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(25, 35)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight
rightLane = rightLaneSec.lane
whiteAdvSpawnPt = new OrientedPoint in rightLaneSec.centerline
redAdvSpawnPt = new OrientedPoint in rightLaneSec.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior DarkAdvBehavior():
    while True:
        take SetBrakeAction(1)

darkAdv = new Car at darkAdvSpawnPt,
    with color Color(0, 0, 0),
    with behavior DarkAdvBehavior()

behavior DarkAdv2Behavior():
    while True:
        take SetBrakeAction(1)

darkAdv2 = new Car at darkAdv2SpawnPt,
    with color Color(0, 0, 0),
    with behavior DarkAdv2Behavior()

param WHITE_ADV_SPEED = Range(7, 10)

behavior WhiteAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.WHITE_ADV_SPEED)

whiteAdv = new Car at whiteAdvSpawnPt,
    with color Color(1, 1, 1),
    with behavior WhiteAdvBehavior()

param RED_ADV_SPEED = Range(7, 10)

behavior RedAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.RED_ADV_SPEED)

redAdv = new Car at redAdvSpawnPt,
    with color Color(1, 0, 0),
    with behavior RedAdvBehavior()

EGO_INT_MIN = 10
EGO_INT_MAX = 30
DARK_ADV_MIN = 10
DARK_ADV_MAX = 20
DARK_ADV2_MIN = 25
DARK_ADV2_MAX = 35
TERM_DIST = 70

require EGO_INT_MIN <= (distance from ego to intersection) <= EGO_INT_MAX
require DARK_ADV_MIN <= (distance from ego to darkAdv) <= DARK_ADV_MAX
require DARK_ADV2_MIN <= (distance from ego to darkAdv2) <= DARK_ADV2_MAX
terminate when (distance to egoSpawnPt) > TERM_DIST
