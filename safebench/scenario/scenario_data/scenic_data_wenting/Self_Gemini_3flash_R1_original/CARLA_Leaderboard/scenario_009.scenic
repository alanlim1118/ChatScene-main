description = "Ego vehicle avoids a cut-in from a vehicle from a static traffic lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param ADV_FORWARD_DIST = Range(12, 18)

laneSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section.isForward:
            if (section._laneToLeft is not None and section._laneToLeft.isForward) or (section._laneToRight is not None and section._laneToRight.isForward):
                laneSections.append(section)

egoSection = Uniform(*laneSections)
egoSpawnPt = new OrientedPoint in egoSection.centerline

if egoSection._laneToLeft is not None and egoSection._laneToLeft.isForward:
    adjSection = egoSection._laneToLeft
else:
    adjSection = egoSection._laneToRight

basePtInAdjLane = adjSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from basePtInAdjLane for globalParameters.ADV_FORWARD_DIST

param EGO_SPEED = Range(8, 12)
param SAFETY_DIST = Range(10, 15)

behavior EgoBehavior(speed, safety_dist):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFETY_DIST)

param ADV_SPEED = Range(7, 10)
param CUT_IN_TRIGGER_DIST = Range(12, 15)

behavior AdversaryBehavior(speed, target_section):
    do FollowLaneBehavior(target_speed=speed) until (distance to ego) < globalParameters.CUT_IN_TRIGGER_DIST
    do LaneChangeBehavior(laneSectionToSwitch=target_section, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with heading advSpawnPt.heading,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED, egoSection)

TERM_DIST = 50

require 12 <= (distance from egoSpawnPt to advSpawnPt) <= 18

terminate when ego intersects adversary
terminate when (distance to egoSpawnPt) > TERM_DIST