description = "Ego vehicle avoids collision from an adjacent vehicle's cut-in maneuver through braking or deceleration."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.yamaha.yzf'
param weather = 'ClearNoon'

laneSectionsWithAdj = []
for lane in network.lanes:
    for section in lane.sections:
        if section.isForward and (section._laneToLeft is not None or section._laneToRight is not None):
            laneSectionsWithAdj.append(section)

egoSection = Uniform(*laneSectionsWithAdj)
egoInitLane = egoSection.lane
egoSpawnPt = new OrientedPoint in egoSection.centerline

possibleAdjSections = []
if egoSection._laneToLeft is not None:
    possibleAdjSections.append(egoSection._laneToLeft)
if egoSection._laneToRight is not None:
    possibleAdjSections.append(egoSection._laneToRight)

advSection = Uniform(*possibleAdjSections)
advInitLane = advSection.lane

advBasePt = advSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advSection.orientation from advBasePt for Range(10, 15)

egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

param OPT_EGO_SPEED = Range(10, 15)
param OPT_BRAKE_DIST = Range(10, 15)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(15, 20)
param OPT_CUT_IN_DIST = Range(15, 20)

behavior AdversaryBehavior(target_lane_section):
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego) > globalParameters.OPT_CUT_IN_DIST
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_section, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(egoSection)

require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 15
terminate when (ego intersects adversary)
terminate when (distance from ego to adversary) > 30