description = "Vehicle backs up in urban area (25 mph), then departs road into driveway/alley."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

drivewayIntersections = filter(lambda i: i.is3Way, network.intersections)

eligibleSections = []
for inter in drivewayIntersections:
    for lane in inter.incomingLanes:
        for sec in lane.sections:
            if sec._laneToRight is not None:
                eligibleSections.append(sec)

egoSection = Uniform(*eligibleSections)
egoInitLane = egoSection.lane
egoSpawnPt = new OrientedPoint in egoSection.centerline

param EGO_SPEED = 11.17
param BACKUP_THROTTLE = 0.2
param BACKUP_DURATION = 3

behavior EgoBehavior(target_speed, backup_throttle, backup_duration, target_lane):
    take SetReverseAction(True)
    take SetThrottleAction(backup_throttle)
    wait for backup_duration seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1.0)
    wait until self.speed < 0.1
    take SetBrakeAction(0)
    take SetReverseAction(False)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED,
        globalParameters.BACKUP_THROTTLE,
        globalParameters.BACKUP_DURATION,
        egoSection._laneToRight
    )

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()

terminate when (distance from ego to egoSpawnPt) > 20