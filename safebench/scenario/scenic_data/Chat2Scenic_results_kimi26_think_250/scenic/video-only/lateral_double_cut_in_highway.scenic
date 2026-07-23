description = "Ego vehicle performs emergency braking on a highway to avoid a rear-end collision after a black SUV cuts in and stops abruptly."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoLane = Uniform(*filter(lambda l: len(l.sections) > 0 and l.sections[0]._laneToRight is not None, network.lanes))
advLane = egoLane.sections[0]._laneToRight.lane
egoSpawnPt = new OrientedPoint in egoLane.centerline
advSpawnPt = new OrientedPoint in advLane.centerline

param EGO_SPEED = Range(15, 25)
param EGO_SAFETY_DIST = Range(10, 15)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.EGO_SAFETY_DIST):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(
            laneSectionToSwitch=leftLaneSec,
            target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=0)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior()

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 60
terminate when (distance from ego to adversary) <= 5