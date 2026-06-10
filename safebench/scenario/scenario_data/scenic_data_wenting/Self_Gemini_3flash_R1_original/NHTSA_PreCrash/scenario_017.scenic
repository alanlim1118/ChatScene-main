description = "Vehicle drifts and encroaches into an oncoming vehicle while going straight in a rural non-junction area with a high speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

oncomingSections = []
for lane in network.lanes:
    if lane.centerline.length > 150:
        for sec in lane.sections:
            if sec._laneToLeft and sec._laneToLeft.isForward != sec.isForward:
                oncomingSections.append(sec)

egoSection = Uniform(*oncomingSections)
advSection = egoSection._laneToLeft

egoSpawnPt = new OrientedPoint on egoSection.centerline

parallelVec = advSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advSection.orientation from parallelVec for Range(-100, -60)

egoTrajectory = [egoSection.lane]
advTrajectory = [advSection.lane]

param EGO_SPEED = Range(15, 20)

behavior EgoBehavior(trajectory, target_lane_sec):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory) for Range(3, 5) seconds
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, is_oppositeTraffic=True, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, is_oppositeTraffic=True)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, advSection)

param ADV_SPEED = Range(15, 20)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

require 60 <= (distance from egoSpawnPt to advSpawnPt) <= 100

terminate when (distance from ego to egoSpawnPt) > 150
terminate after 15 seconds