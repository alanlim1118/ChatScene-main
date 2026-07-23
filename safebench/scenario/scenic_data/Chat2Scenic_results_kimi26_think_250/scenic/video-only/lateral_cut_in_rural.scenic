description = "Ego vehicle on a narrow wet rural road in adverse weather is hit by an oncoming vehicle swerving into its lane, causing a side-swipe collision."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'SoftRainNoon'

initLane = Uniform(*filter(lambda lane:
	all([sec._laneToLeft is not None and sec._laneToLeft.isForward is not sec.isForward for sec in lane.sections]),
	network.lanes))
egoSpawnPt = new OrientedPoint in initLane.centerline
egoLaneSec = Uniform(*initLane.sections)
advLane = egoLaneSec._laneToLeft.lane
advSpawnPt = new OrientedPoint in advLane.centerline
debrisSpawnPt = new OrientedPoint right of advSpawnPt by Range(2, 4)

param OPT_EGO_SPEED = Range(8, 12)
param OPT_EGO_SAFETY_DIST = Range(10, 15)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_EGO_SAFETY_DIST):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(10, 14)
param OPT_AVOID_DIST = Range(5, 10)
param OPT_ADV_STEER = Range(-1.0, -0.6)

behavior SwerveAction(steer):
    take SetSteerAction(steer)
    take SetThrottleAction(0.5)
    wait for 1 steps

behavior AdvSwerveBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to debrisSpawnPt) < globalParameters.OPT_AVOID_DIST
    do SwerveAction(globalParameters.OPT_ADV_STEER) for 1.5 seconds
    take SetSteerAction(0)
    do ConstantThrottleBehavior(0.5)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvSwerveBehavior()

debris = new Debris at debrisSpawnPt

INIT_DIST = 40
TERM_DIST = 100

require (distance from ego to adversary) > INIT_DIST
terminate when (distance to egoSpawnPt) > TERM_DIST