description = "Ego vehicle avoids rear-end collision with slower vehicle performing a lane change into its path."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
advLaneSec = egoLaneSec._laneToRight

egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

refPtOnAdvLane = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advLaneSec.orientation from refPtOnAdvLane for Range(15, 25)

param EGO_SPEED = Range(15, 20)
param EGO_BRAKE_THRESHOLD = Range(12, 18)

behavior EgoBehavior(target_speed, threshold):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, threshold):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_BRAKE_THRESHOLD)

param ADV_SPEED = Range(10, 15)
param ADV_THRESHOLD = Range(20, 30)

behavior AdversaryBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < threshold
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED, globalParameters.ADV_THRESHOLD)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when (distance from ego to egoSpawnPt) > 100
terminate after 30 seconds