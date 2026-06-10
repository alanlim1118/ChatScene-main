description = "Vehicle A attempts an unsafe pass of Vehicle B, resulting in a collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoInitLaneSec = Uniform(*laneSecsWithLeftLane)
advSpawnPt = new OrientedPoint in egoInitLaneSec.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(10, 15)

param OPT_EGO_SPEED = Range(20, 25)
param OPT_PASS_DISTANCE = Range(5, 8)

behavior EgoBehavior(speed, pass_dist, target_lane):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToAnyCars(self, pass_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_PASS_DISTANCE, egoInitLaneSec._laneToLeft)

param OPT_ADV_SPEED = Range(10, 12)

behavior AdversaryBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED)

EGO_ADV_DIST = [10, 15]

require EGO_ADV_DIST[0] <= (distance from egoSpawnPt to advSpawnPt) <= EGO_ADV_DIST[1]
terminate when ego intersects adv