description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearNoon"
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_START_DIST = Range(15, 25)
param OPT_LEADING_DIST = Range(10, 20)

laneSecsWithLeftRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneSec = egoLaneSec._laneToLeft
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_ADV_START_DIST

rightLaneSec = egoLaneSec._laneToRight
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
convergePt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_LEADING_DIST

behavior EgoBehavior(speed, brake_threshold):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn egoLaneSec,
	with blueprint MODEL,
	with behavior EgoBehavior(10, 5)

param OPT_ADV_SPEED = Range(15, 20)
param OPT_MERGE_TRIGGER_DIST = Range(20, 25)

behavior AdvBehavior(target_speed, target_lane_sec, trigger_dist):
	do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego) < trigger_dist
	do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=target_speed)
	do FollowLaneBehavior(target_speed=target_speed)

adv = new Car at advSpawnPt,
	with regionContainedIn leftLaneSec,
	with blueprint MODEL,
	with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, egoLaneSec, globalParameters.OPT_MERGE_TRIGGER_DIST)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLights()
require 15 <= (distance from egoSpawnPt to intersection) <= 35
require 5 <= (distance from advSpawnPt to intersection) <= 15

terminate when (distance from ego to convergePt) > 60 and (distance from adv to convergePt) > 60
terminate after 30 seconds