description = "Ego vehicle turning right; adversarial car ahead on right reverses abruptly."
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
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

adjLaneSec = egoLaneSec._laneToRight
sideVector = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following adjLaneSec.orientation from sideVector for Range(10, 20)

egoTrajectory = [egoLaneSec.lane]
advTrajectory = [adjLaneSec.lane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_TARGET_SPEED = Range(5, 8)
param ADV_REVERSE_SPEED = Range(2, 5)
param REVERSE_TRIGGER_DIST = Range(10, 15)
param REVERSE_DURATION = Range(2, 4)

behavior WaitBehavior():
    while True:
        wait

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_TARGET_SPEED, trajectory=trajectory) until (distance from self to ego) < globalParameters.REVERSE_TRIGGER_DIST
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    take SetReverseAction(True)
    take SetSpeedAction(globalParameters.ADV_REVERSE_SPEED)
    do WaitBehavior() for globalParameters.REVERSE_DURATION seconds
    terminate

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

param SUCCESS_DIST = 60
param SCENARIO_TIMEOUT = 40

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 20
terminate when (distance from ego to egoSpawnPt) > globalParameters.SUCCESS_DIST
terminate after globalParameters.SCENARIO_TIMEOUT seconds