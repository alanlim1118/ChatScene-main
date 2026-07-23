description = "Ego vehicle travels straight in the middle lane while an adversary cuts ahead from the left lane toward the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsMiddle = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsMiddle.append(laneSec)

egoLaneSec = Uniform(*laneSecsMiddle)
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

param OPT_ADV_AHEAD_DIST = Range(8, 20)
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_ADV_AHEAD_DIST

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(8, 14)

behavior AdversaryBehavior():
    do LaneChangeBehavior(egoLaneSec, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require 8 <= (distance from egoSpawnPt to AdvSpawnPt) <= 25