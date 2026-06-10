description = "Lead vehicle performs an emergency lane change to avoid a stopped car at low TTC, creating a high-urgency late reveal for ego."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_TO_LEAD_DIST = Range(10, 15)
param LEAD_TO_STOPPED_DIST = Range(15, 25)
forwardSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section.isForward:
            forwardSections.append(section)
egoLaneSec = Uniform(*forwardSections)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.EGO_TO_LEAD_DIST
stoppedSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.LEAD_TO_STOPPED_DIST

param EGO_SPEED = Range(12, 15)
param EGO_BRAKE_THRESHOLD = 12

behavior EgoBehavior(speed, brake_threshold):
    do FollowLaneBehavior(target_speed=speed)


ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_BRAKE_THRESHOLD)

param LEAD_SPEED = Range(12, 15)
param LEAD_THRESHOLD = 10

behavior LeadBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, threshold)
    if self.laneSection._laneToLeft:
        do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=speed)
    elif self.laneSection._laneToRight:
        do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

stopped = new Car at stoppedSpawnPt,
    with blueprint MODEL

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior(globalParameters.LEAD_SPEED, globalParameters.LEAD_THRESHOLD)


require 15 <= (distance from leadSpawnPt to stoppedSpawnPt) <= 25
terminate when (distance from ego to egoSpawnPt) > 80