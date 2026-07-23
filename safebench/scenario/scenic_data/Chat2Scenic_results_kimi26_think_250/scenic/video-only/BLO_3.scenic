description = "Ego vehicle is obstructed by dense traffic congestion between two intersections on a straight multi-lane road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 3 and all(sec._laneToLeft is not None and sec._laneToRight is not None for sec in r.forwardLanes.lanes[1].sections), network.roads))
egoLane = road.forwardLanes.lanes[1]
egoSpawnPt = new OrientedPoint in egoLane.centerline
leftLane = egoLane.sectionAt(egoSpawnPt).laneToLeft.lane
rightLane = egoLane.sectionAt(egoSpawnPt).laneToRight.lane
aheadPt1 = new OrientedPoint ahead of egoSpawnPt by 10
aheadPt2 = new OrientedPoint ahead of egoSpawnPt by 15
clusterSameSpawnPt1 = aheadPt1
clusterSameSpawnPt2 = aheadPt2
clusterLeftSpawnPt1 = new OrientedPoint left of aheadPt1 by 3.5
clusterLeftSpawnPt2 = new OrientedPoint left of aheadPt2 by 3.5
clusterRightSpawnPt1 = new OrientedPoint right of aheadPt1 by 3.5
clusterRightSpawnPt2 = new OrientedPoint right of aheadPt2 by 3.5

param EGO_SPEED = Range(8, 11)
param CONGESTION_DIST = Range(9, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToObjsInLane(self, globalParameters.CONGESTION_DIST)
    while True:
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at clusterSameSpawnPt1,
    with blueprint MODEL

adversary2 = new Car at clusterSameSpawnPt2,
    with blueprint MODEL

adversary3 = new Car at clusterLeftSpawnPt1,
    with blueprint MODEL

INIT_DIST = 5
TERM_DIST = 100

require (distance to adversary) > INIT_DIST
terminate when (distance to adversary2) > TERM_DIST