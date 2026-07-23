"""Scenario Description:

Under misty weather conditions, the ego vehicle travels straight through an urban, multi-lane four-way intersection. As the ego vehicle approaches the junction, it navigates past several crossing vehicles, including one turning left from the left arm, another passing straight from the left arm, a third passing straight from the opposite arm, and a fourth passing straight from the right arm. Furthermore, three pedestrians are observed crossing the intersection, with one pedestrian crossing from the far side and two pedestrians crossing from the near side relative to the ego vehicle.

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

EGO_INIT_DIST = [30, 40]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [20, 35]
param ADV_SPEED = VerifaiRange(5, 8)

param PED_SPEED = VerifaiRange(0.8, 1.4)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 3
TERM_DIST = 80

#################################
# WEATHER                       #
#################################

param weather = WeatherConditions(
    fog_density=0.4,
    fog_distance=10,
    wetness=0.3,
    cloudiness=0.8
)

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

behavior PedCrossingBehavior(startPt, endPt):
    do WalkTowardsBehavior(target=endPt, speed=globalParameters.PED_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Determine lateral and opposite lanes relative to ego
leftLanes = filter(lambda l: l is not egoInitLane and 
                   abs(relativeHeading(l.centerline[0], egoInitLane.centerline[0]) - 1.5708) < 0.5,
                   intersection.incomingLanes)
rightLanes = filter(lambda l: l is not egoInitLane and 
                    abs(relativeHeading(l.centerline[0], egoInitLane.centerline[0]) + 1.5708) < 0.5,
                    intersection.incomingLanes)
oppositeLanes = filter(lambda l: l is not egoInitLane and 
                       abs(abs(relativeHeading(l.centerline[0], egoInitLane.centerline[0])) - 3.14159) < 0.5,
                       intersection.incomingLanes)

# Adversary 1: Left turn from left arm
advLeftTurnLane = Uniform(*leftLanes) if leftLanes else egoInitLane
advLeftTurnManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advLeftTurnLane.maneuvers))
advLeftTurnTraj = [advLeftTurnLane, advLeftTurnManeuver.connectingLane, advLeftTurnManeuver.endLane]
advLeftTurnSpawn = new OrientedPoint in advLeftTurnLane.centerline

# Adversary 2: Straight from left arm
advLeftStraightLane = Uniform(*leftLanes) if leftLanes else egoInitLane
advLeftStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advLeftStraightLane.maneuvers))
advLeftStraightTraj = [advLeftStraightLane, advLeftStraightManeuver.connectingLane, advLeftStraightManeuver.endLane]
advLeftStraightSpawn = new OrientedPoint in advLeftStraightLane.centerline

# Adversary 3: Straight from opposite arm
advOppositeLane = Uniform(*oppositeLanes) if oppositeLanes else egoInitLane
advOppositeManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advOppositeLane.maneuvers))
advOppositeTraj = [advOppositeLane, advOppositeManeuver.connectingLane, advOppositeManeuver.endLane]
advOppositeSpawn = new OrientedPoint in advOppositeLane.centerline

# Adversary 4: Straight from right arm
advRightLane = Uniform(*rightLanes) if rightLanes else egoInitLane
advRightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advRightLane.maneuvers))
advRightTraj = [advRightLane, advRightManeuver.connectingLane, advRightManeuver.endLane]
advRightSpawn = new OrientedPoint in advRightLane.centerline

# Pedestrian crossing regions
nearSideRegion = Region.union(*[lane.sidewalk for lane in [egoInitLane] if hasattr(lane, 'sidewalk')])
farSideRegion = Region.union(*[lane.sidewalk for lane in list(oppositeLanes) if hasattr(lane, 'sidewalk')])

# Near side pedestrians (2)
pedNear1Start = new OrientedPoint in nearSideRegion
pedNear1End = new OrientedPoint in nearSideRegion
require distance from pedNear1Start to pedNear1End > 5

pedNear2Start = new OrientedPoint in nearSideRegion
pedNear2End = new OrientedPoint in nearSideRegion
require distance from pedNear2Start to pedNear2End > 5
require distance from pedNear1Start to pedNear2Start > 3

# Far side pedestrian (1)
pedFarStart = new OrientedPoint in farSideRegion
pedFarEnd = new OrientedPoint in farSideRegion
require distance from pedFarStart to pedFarEnd > 5

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adv1 = new Car at advLeftTurnSpawn,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advLeftTurnTraj)

adv2 = new Car at advLeftStraightSpawn,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advLeftStraightTraj)

adv3 = new Car at advOppositeSpawn,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advOppositeTraj)

adv4 = new Car at advRightSpawn,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advRightTraj)

ped1 = new Pedestrian at pedNear1Start,
    with blueprint PED_MODEL,
    with behavior PedCrossingBehavior(pedNear1Start, pedNear1End)

ped2 = new Pedestrian at pedNear2Start,
    with blueprint PED_MODEL,
    with behavior PedCrossingBehavior(pedNear2Start, pedNear2End)

ped3 = new Pedestrian at pedFarStart,
    with blueprint PED_MODEL,
    with behavior PedCrossingBehavior(pedFarStart, pedFarEnd)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv3 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv4 to intersection) <= ADV_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST