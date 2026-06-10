description = "Ego vehicle forced to change lanes due to slow, multi-lane blocking adversarial vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param ADV_AHEAD_DIST = Range(30, 45)

laneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToLeft is not None:
            if laneSec._laneToRight.isForward and laneSec._laneToLeft.isForward:
                laneSecs.append(laneSec)

egoLaneSec = Uniform(*laneSecs)
rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint on egoLaneSec.centerline
rightLaneRefPt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from rightLaneRefPt for globalParameters.ADV_AHEAD_DIST

egoTrajectory = [egoLaneSec.lane, leftLaneSec.lane]
advTrajectory = [rightLaneSec.lane, egoLaneSec.lane]

param EGO_SPEED = 10
param LC_THRESHOLD = 20

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToObjsInLane(self, globalParameters.LC_THRESHOLD)
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(1, 3)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior()

param TERMINATION_DIST = 70
param MAX_TIME = 40

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLights()
require ego can see adv
require 30 <= (distance from egoSpawnPt to advSpawnPt) <= 50

terminate when (distance from ego to adv) > globalParameters.TERMINATION_DIST
terminate after globalParameters.MAX_TIME seconds