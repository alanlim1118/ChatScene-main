description = "Ego vehicle yields to two adversaries on the main highway while merging from an on-ramp at night, then follows them."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param OPT_EGO_SPEED = Range(8, 12)
param OPT_YIELD_DIST = Range(12, 18)

behavior EgoBehavior():
    while True:
        try:
            do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
        interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_YIELD_DIST):
            take SetThrottleAction(0)
            take SetBrakeAction(1)
            while withinDistanceToAnyCars(self, globalParameters.OPT_YIELD_DIST):
                wait

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param ADV2_SPEED = Range(7, 10)

behavior ForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior ForwardBehavior()

param EGO_ADV_MIN = 10
param EGO_ADV_MAX = 40
param TERM_DIST = 100

require (distance from ego to adversary) >= globalParameters.EGO_ADV_MIN
require (distance from ego to adversary) <= globalParameters.EGO_ADV_MAX
require (distance from ego to adversary2) >= globalParameters.EGO_ADV_MIN
require (distance from ego to adversary2) <= globalParameters.EGO_ADV_MAX

terminate when (distance to egoSpawnPt) > globalParameters.TERM_DIST