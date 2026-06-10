description = "Vehicle A misjudges distance of Vehicle B during lane change, leading to a rear-end collision."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)

adjSection = Uniform(*filter(lambda s: s is not None and s.isForward, [egoSection._laneToLeft, egoSection._laneToRight]))

param OPT_DISTANCE = Range(-25, -15)

projectedPt = adjSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following adjSection.orientation from projectedPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoSection,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(15, 25)
param OPT_BRAKE_DISTANCE = Range(8, 15)
param OPT_BRAKE_FORCE = Range(0.4, 0.7)

behavior AdversarialBehavior(speed, brake_dist, brake_val):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < brake_dist
    while True:
        take SetBrakeAction(brake_val)

advAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversarialBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_BRAKE_DISTANCE, globalParameters.OPT_BRAKE_FORCE)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when ego intersects advAgent
