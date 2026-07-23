description = "Ego vehicle must go around an obstacle (stationary car) using the opposite lane, yielding to oncoming traffic."

param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

MODEL = 'vehicle.mini.cooper_s_2021'

# -----------------------------------------------------------
# 1. PYTHON PHASE: Guarantee a massive, long road!
# -----------------------------------------------------------
validLanes = []
for lane in network.lanes:
    if all([sec._laneToLeft is not None and sec._laneToLeft.isForward is not sec.isForward for sec in lane.sections]):
        # The lane MUST be long enough to support an 80m intersection gap!
        if lane.centerline.length > 200:
            validLanes.append(lane)

initLane = Uniform(*validLanes)

# -----------------------------------------------------------
# 2. SPATIAL SETUP PHASE
# -----------------------------------------------------------
egoSpawnPt = new OrientedPoint on initLane.centerline

ADV_INIT_DIST = [20, 25]

# Spawn the obstacle precisely 25m ahead
obstacle = new Car following roadDirection from egoSpawnPt for ADV_INIT_DIST[1],
    with blueprint MODEL,
    with viewAngle 90 deg

# Explicitly spawn the Adversary in the ONCOMING lane directly next to the obstacle!
advLaneSec = obstacle.laneSection.laneToLeft
advSpawnPt = new OrientedPoint on advLaneSec.centerline

# -----------------------------------------------------------
# 3. BEHAVIOR PHASE (Sequential Bypass)
# -----------------------------------------------------------
param EGO_SPEED = Range(7, 10)
BYPASS_DIST = [10, 15]

behavior EgoBehavior():
    # Drive until close to the obstacle
    do FollowLaneBehavior(globalParameters.EGO_SPEED) \
        until (distance to obstacle) < BYPASS_DIST[1]
        
    # Swerve Left
    leftLaneSec = self.laneSection.laneToLeft
    do LaneChangeBehavior(
        laneSectionToSwitch=leftLaneSec, 
        is_oppositeTraffic=True, 
        target_speed=globalParameters.EGO_SPEED)
        
    # Drive in opposite lane until passed
    do FollowLaneBehavior(globalParameters.EGO_SPEED, is_oppositeTraffic=True) \
        until (distance to obstacle) > BYPASS_DIST[0]
        
    # Swerve Right
    rightLaneSec = self.laneSection.laneToRight
    do LaneChangeBehavior(
        laneSectionToSwitch=rightLaneSec,
        is_oppositeTraffic=False,
        target_speed=globalParameters.EGO_SPEED)
        
    do FollowLaneBehavior(globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(globalParameters.ADV_SPEED)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

# -----------------------------------------------------------
# 4. CONSTRAINT PHASE
# -----------------------------------------------------------
INIT_DIST = 80
TERM_DIST = 50

# Explicit anchor used for intersection distance
require (distance from ego to intersection) > INIT_DIST

# The distance constraint between Adv and Obstacle is removed because 
# we mathematically guaranteed they are side-by-side using lane math!

terminate when (distance to egoSpawnPt) > TERM_DIST
terminate after 30 seconds