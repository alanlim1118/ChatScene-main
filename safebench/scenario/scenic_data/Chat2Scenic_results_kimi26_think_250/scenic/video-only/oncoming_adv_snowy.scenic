description = "Ego vehicle drives on a snow-covered rural road as an oncoming car loses traction and skids into its path."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

param OPT_ADV_DIST = Range(15, 30)
param OPT_AHEAD_DIST = Range(40, 70)

laneSecsWithOppLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithOppLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithOppLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

oppLaneSec = egoLaneSec._laneToLeft
oppLanePt = oppLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from oppLanePt for globalParameters.OPT_ADV_DIST

aheadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_AHEAD_DIST

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(9, 10)
param SKID_DELAY = Range(2, 4)
param SKID_STEER = 1.0

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for globalParameters.SKID_DELAY
	while True:
		take RegulatedControlAction(throttle=-0.5, steer=globalParameters.SKID_STEER, past_steer=globalParameters.SKID_STEER, max_throttle=0.5, max_brake=0.5, max_steer=1.0)

adversary = new Car at advSpawnPt,
	facing away from aheadSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param AHEAD_SPEED = Range(7, 9)

behavior AheadCarBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.AHEAD_SPEED)

aheadCar = new Car at aheadSpawnPt,
	with blueprint MODEL,
	with behavior AheadCarBehavior()

MIN_EGO_ADV_DIST = 10
MAX_EGO_ADV_DIST = 35
MIN_EGO_AHEAD_DIST = 35
MAX_EGO_AHEAD_DIST = 75
TERM_DIST = 80

require MIN_EGO_ADV_DIST <= (distance to adversary) <= MAX_EGO_ADV_DIST
require MIN_EGO_AHEAD_DIST <= (distance to aheadCar) <= MAX_EGO_AHEAD_DIST
terminate when (distance to egoSpawnPt) > TERM_DIST