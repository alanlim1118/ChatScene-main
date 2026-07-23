"""Scenario Description:

Under dark weather conditions, the ego vehicle travels straight into an urban, multi-lane four-way intersection. As the vehicle proceeds, it suddenly brakes to yield to a pedestrian crossing the intersection from left to right, while simultaneously an adversary vehicle approaching from the left arm enters the intersection.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
param weather = "Night"
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

param PED_SPEED = VerifaiRange(1.0, 2.0)
param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior PedestrianCrossingBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.PED_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego setup: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Pedestrian crossing from left to right relative to ego
refPt = new OrientedPoint at intersection.polygon.centroid, facing egoSpawnPt.heading
pedStart = left of refPt by 8
pedEnd = right of refPt by 8
pedTrajectory = PolylineRegion([pedStart, pedEnd])

# Adversary setup: approaching from the left arm
# Identify opposite lane to exclude
oppositeManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oppositeLane = oppositeManeuver.startLane

# Candidate lanes are the remaining two incoming lanes
candidateLanes = [l for l in intersection.incomingLanes if l is not egoInitLane and l is not oppositeLane]

# Determine which candidate is the left arm using cross product
center = intersection.polygon.centroid
egoEnd = egoInitLane.centerline.points[-1]
egoOut = egoEnd - center

leftLanes = []
for lane in candidateLanes:
    laneEnd = lane.centerline.points[-1]
    laneOut = laneEnd - center
    cross = egoOut.x * laneOut.y - egoOut.y * laneOut.x
    if cross < 0:
        leftLanes.append(lane)

advInitLane = Uniform(*leftLanes)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

pedestrian = new Pedestrian at pedStart,
    with behavior PedestrianCrossingBehavior(pedTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST