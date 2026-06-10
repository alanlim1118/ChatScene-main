description = "Ego vehicle follows a slow lead vehicle on a straight highway, then performs a lane change to overtake it."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSectionsWithNeighbors = []
for lane in network.lanes:
    for section in lane.sections:
        if section.isForward:
            if section._laneToLeft is not None:
                laneSectionsWithNeighbors.append(section)
            elif section._laneToRight is not None:
                laneSectionsWithNeighbors.append(section)

egoSection = Uniform(*laneSectionsWithNeighbors)
egoSpawnPt = new OrientedPoint in egoSection.centerline

leadSpawnPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for Range(20, 30)

if egoSection._laneToLeft is not None:
    targetSection = egoSection._laneToLeft
else:
    targetSection = egoSection._laneToRight

egoInitLane = egoSection.lane
targetLane = targetSection.lane
leadLane = egoSection.lane

param OPT_EGO_SPEED = Range(10, 15)
param OPT_OVERTAKE_DIST = Range(15, 20)

behavior EgoBehavior(speed, overtake_dist, target_lane_sec):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, overtake_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoSection,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_OVERTAKE_DIST,
        targetSection
    )

param OPT_NPC_SPEED = globalParameters.OPT_EGO_SPEED - 5

behavior NPCBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

npcCar = new NPCCar at leadSpawnPt,
    with behavior NPCBehavior(globalParameters.OPT_NPC_SPEED)

require 20 <= (distance from egoSpawnPt to leadSpawnPt) <= 30
require ego can see npcCar
terminate after 30 seconds