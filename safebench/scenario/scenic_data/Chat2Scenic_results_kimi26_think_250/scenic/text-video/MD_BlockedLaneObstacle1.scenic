description = "Ego vehicle blocked by stopped adversaries at a four-way intersection while adjacent lane traffic flows freely."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
darkAdvSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 20)
darkAdv2SpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(25, 35)
egoLaneSec = Uniform(*egoInitLane.sections)
rightLaneSec = egoLaneSec._laneToRight
rightLane = rightLaneSec.lane
whiteAdvSpawnPt = new OrientedPoint in rightLaneSec.centerline
redAdvSpawnPt = new OrientedPoint in rightLaneSec.centerline

param OPT_EGO_SPEED = Range(2, 5)
param OPT_BRAKE_DIST = Range(8, 12)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        while True:
            take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

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