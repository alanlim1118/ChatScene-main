description = "Ego vehicle travels straight while an adversarial vehicle accelerates and passes on the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

eligibleSections = []
for road in network.roads:
    if road.forwardLanes is not None and len(road.forwardLanes.lanes) == 2:
        for lane in road.forwardLanes.lanes:
            for sec in lane.sections:
                if sec._laneToRight is not None:
                    eligibleSections.append(sec)
egoLaneSec = Uniform(*eligibleSections)
advLaneSec = egoLaneSec._laneToRight
egoLane = egoLaneSec.lane
advLane = advLaneSec.lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

behavior EgoBehavior():
    do FollowLaneBehavior()

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior AdvBehavior():
    do AccelerateForwardBehavior()

adversarial = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()