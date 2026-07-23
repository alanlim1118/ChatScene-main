"""Scenario Description:

In a top-down view of a four-way intersection, the ego vehicle attempts to proceed straight through the junction but remains positioned in the center lane. It encounters a complex traffic situation involving multiple adversary vehicles approaching and crossing from various arms of the intersection (north, south, and east). Additionally, pedestrians are visible at the corners of the junction. Due to these intersecting vehicle movements and pedestrian presence, the ego vehicle is required to yield, waiting in the middle of the intersection to carefully monitor cross-traffic and pedestrian paths before safely resuming its straight trajectory.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'
PED_MODEL = 'walker.pedestrian.0001'

EGO_INIT_DIST = [25, 35]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.8, 1.0)
param EGO_RESUME_SPEED = VerifaiRange(5, 8)

ADV_INIT_DIST = [20, 30]
param ADV_SPEED = VerifaiRange(6, 10)

param SAFETY_DIST = VerifaiRange(12, 18)
CRASH_DIST = 4
TERM_DIST = 80
YIELD_WAIT_TIME = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoYieldBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
        do WaitBehavior() for YIELD_WAIT_TIME seconds
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_RESUME_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior WaitBehavior():
    while True:
        wait

behavior PedestrianCornerBehavior():
    while True:
        take SetWalkingSpeedAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary from conflicting straight maneuver (opposite direction)
conflictingStraightManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers)
advOppositeManeuver = Uniform(*conflictingStraightManeuvers) if conflictingStraightManeuvers else None
advOppositeLane = advOppositeManeuver.startLane if advOppositeManeuver else None
advOppositeSpawnPt = new OrientedPoint in advOppositeLane.centerline if advOppositeLane else None

# Adversary from left cross-traffic (straight across ego's path)
leftCrossManeuvers = filter(lambda m: 
    m.type is ManeuverType.STRAIGHT and m.startLane is not egoInitLane and 
    (advOppositeLane is None or m.startLane is not advOppositeLane),
    egoManeuver.conflictingManeuvers)
advLeftManeuver = Uniform(*leftCrossManeuvers) if leftCrossManeuvers else None
advLeftLane = advLeftManeuver.startLane if advLeftManeuver else None
advLeftSpawnPt = new OrientedPoint in advLeftLane.centerline if advLeftLane else None

# Adversary from right cross-traffic
rightCrossManeuvers = filter(lambda m:
    m.type is ManeuverType.STRAIGHT and m.startLane is not egoInitLane and
    (advOppositeLane is None or m.startLane is not advOppositeLane) and
    (advLeftLane is None or m.startLane is not advLeftLane),
    egoManeuver.conflictingManeuvers)
advRightManeuver = Uniform(*rightCrossManeuvers) if rightCrossManeuvers else None
advRightLane = advRightManeuver.startLane if advRightManeuver else None
advRightSpawnPt = new OrientedPoint in advRightLane.centerline if advRightLane else None

# Pedestrians at intersection corners
cornerRegions = intersection.cornerRegions if hasattr(intersection, 'cornerRegions') else intersection.region
pedSpawnPts = [new OrientedPoint in cornerRegions for _ in range(4)]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoYieldBehavior(egoTrajectory)

# Adversary vehicles from up to three directions
if advOppositeSpawnPt is not None:
    adv1 = new Car at advOppositeSpawnPt,
        with blueprint MODEL,
        with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, 
            trajectory=[advOppositeLane, advOppositeManeuver.connectingLane, advOppositeManeuver.endLane])
    require ADV_INIT_DIST[0] <= (distance from adv1 to intersection) <= ADV_INIT_DIST[1]

if advLeftSpawnPt is not None:
    adv2 = new Car at advLeftSpawnPt,
        with blueprint MODEL,
        with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED,
            trajectory=[advLeftLane, advLeftManeuver.connectingLane, advLeftManeuver.endLane])
    require ADV_INIT_DIST[0] <= (distance from adv2 to intersection) <= ADV_INIT_DIST[1]

if advRightSpawnPt is not None:
    adv3 = new Car at advRightSpawnPt,
        with blueprint MODEL,
        with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED,
            trajectory=[advRightLane, advRightManeuver.connectingLane, advRightManeuver.endLane])
    require ADV_INIT_DIST[0] <= (distance from adv3 to intersection) <= ADV_INIT_DIST[1]

# Pedestrians at corners
for pt in pedSpawnPts:
    ped = new Pedestrian at pt,
        with blueprint PED_MODEL,
        with behavior PedestrianCornerBehavior()

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST