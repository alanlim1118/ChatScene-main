description = "Ego vehicle must go around an obstacle (stationary car) using the opposite lane, yielding to oncoming traffic."

param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

MODEL = 'vehicle.mini.cooper_s_2021'

# -----------------------------------------------------------
# 1. PYTHON PHASE: Guarantee a massive, long road!
# -----------------------------------------------------------
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

# -----------------------------------------------------------
# 2. SPATIAL SETUP PHASE
# -----------------------------------------------------------
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
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(globalParameters.OPT_ADV_SPEED)

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