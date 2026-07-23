description = "Ego vehicle swerves left to avoid an obstacle ahead, crossing into the adjacent lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_PROP_DISTANCE = Range(20, 30)
param OPT_GEO_ADV_OFFSET = Range(-2, 2)

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToRight is None and laneSec._laneToLeft.isForward == laneSec.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

propSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_PROP_DISTANCE

leftLaneSec = egoLaneSec._laneToLeft
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_GEO_ADV_OFFSET

param OPT_EGO_SPEED = Range(5, 8)
param OPT_EGO_SWERVE_DIST = Range(10, 15)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to propSpawnPt < globalParameters.OPT_EGO_SWERVE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

prop = new Debris at propSpawnPt

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with behavior AdversaryBehavior()

require (distance from ego to prop) >= 15
require (distance from ego to adversary) >= 2
TERM_DIST = 50
terminate when (distance to egoSpawnPt) > TERM_DIST