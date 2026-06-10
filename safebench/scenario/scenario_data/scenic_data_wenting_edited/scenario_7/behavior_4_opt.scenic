description = "Ego vehicle turns right while an adversarial car behind on the right suddenly accelerates and decelerates."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rightLaneSec = egoLaneSec._laneToRight
adjLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)

param adv_dist_behind = Range(5, 10)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for -param("adv_dist_behind")

egoTrajectory = [egoLaneSec.lane]
advTrajectory = [rightLaneSec.lane]

param ego_speed = Range(6, 9)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=param("ego_speed"))

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param adv_accel_speed = Range(15, 20)
param adv_decel_speed = Range(2, 5)
param accel_duration = Range(3, 6)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=param("adv_accel_speed"), trajectory=advTrajectory) for param("accel_duration") seconds
    do FollowTrajectoryBehavior(target_speed=param("adv_decel_speed"), trajectory=advTrajectory)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLights()

terminate when (distance from ego to egoSpawnPt) > 80
terminate after 60 seconds