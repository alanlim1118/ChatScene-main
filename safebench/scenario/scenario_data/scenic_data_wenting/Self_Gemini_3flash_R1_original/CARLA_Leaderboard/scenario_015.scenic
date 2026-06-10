description = "Ego vehicle encounters a slow moving hazard, requiring braking or maneuvering next to same-direction traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

lanePairs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec._laneToLeft and sec._laneToLeft.isForward == sec.isForward:
            lanePairs.append((sec, sec._laneToLeft))
        if sec._laneToRight and sec._laneToRight.isForward == sec.isForward:
            lanePairs.append((sec, sec._laneToRight))

selectedPair = Uniform(*lanePairs)
egoLaneSec = selectedPair[0]
advLaneSec = selectedPair[1]

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline
propSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for Range(30, 50)

require 5 <= (distance from egoSpawnPt to advSpawnPt) <= 20

param EGO_SPEED = Range(7, 10)
param BRAKE_VAL = 1.0
param MANEUVER_DIST = 25
param SAFETY_DIST = 12

behavior EgoBehavior(speed, maneuver_dist, safety_dist, target_lane):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, maneuver_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED, 
        globalParameters.MANEUVER_DIST, 
        globalParameters.SAFETY_DIST, 
        advLaneSec
    )

param PROP_SPEED = Range(2, 5)

behavior PropBehavior(speed):
	do FollowLaneBehavior(target_speed=speed)

advProp = new Car at propSpawnPt,
	with blueprint MODEL,
	with behavior PropBehavior(globalParameters.PROP_SPEED)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require (distance from egoSpawnPt to propSpawnPt) >= 35
terminate when (distance from ego to egoSpawnPt) > 100