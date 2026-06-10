description = "Ego vehicle attempts a right lane change while an adversarial car blocks its path."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*filter(lambda l: all(sec._laneToRight is not None for sec in l.sections), network.lanes))
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
egoSection = egoInitLane.sectionAt(egoSpawnPt)
advSection = egoSection._laneToRight
advInitLane = advSection.lane
advSpawnPt = new OrientedPoint on advSection.centerline
egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

param EGO_SPEED = 10

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=advSection, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

param MAX_INITIAL_DISTANCE = 15
param TERMINATION_DISTANCE = 100
param TERMINATION_TIME = 60

require (distance from egoSpawnPt to advSpawnPt) <= globalParameters.MAX_INITIAL_DISTANCE
terminate when (distance from ego to egoSpawnPt) >= globalParameters.TERMINATION_DISTANCE
terminate after globalParameters.TERMINATION_TIME seconds