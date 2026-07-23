description = "Ego vehicle approaches a laterally moving object entering from the right forward with converging paths in the right lane."
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
advLaneSec = egoLaneSec._laneToRight
basePt = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advLaneSec.lane.orientation from basePt for Range(10, 30)

behavior EgoBehavior():
    do FollowLaneBehavior()

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior():
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(
        laneSectionToSwitch=leftLaneSec,
        target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

TERM_DIST = 100

require 10 <= (distance from ego to adversary) <= 35
terminate when (distance to egoSpawnPt) > TERM_DIST