description = "Target vehicle merges into ego vehicle's path with a full lateral displacement."

param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

# -----------------------------------------------------------
# 1. PYTHON PHASE: Safely Group Valid Adjacent Lane Pairs
# -----------------------------------------------------------
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoSection = network.laneSectionAt(egoSpawnPt)
adjSection = Uniform(*filter(lambda s: s is not None and s.isForward, [egoSection._laneToLeft, egoSection._laneToRight]))

# -----------------------------------------------------------
# 2. SPATIAL SETUP PHASE (Fixed Math & Spawning Logic)
# -----------------------------------------------------------
# Safely project the Adv spawn point forward in the ADJACENT lane
advBasePt = adjSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advBasePt for Range(20, 30)

# -----------------------------------------------------------
# 3. BEHAVIOR PHASE (Dynamic Target Lane)
# -----------------------------------------------------------
# Ego is faster, so it will catch up to the Adv
param OPT_ADV_SPEED = Range(7, 10) - Range(1, 2)
param OPT_ADV_DISTANCE = Range(10, 15)

behavior AdvBehavior(speed, merge_dist):
    # Drive in adjacent lane until Ego catches up
    do FollowLaneBehavior(target_speed=speed) \
        until (distance to ego) < merge_dist

    # Dynamically grab the Ego's exact current lane section to guarantee a full cut-in!
    do LaneChangeBehavior(laneSectionToSwitch=ego.laneSection, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

# -----------------------------------------------------------
# 4. INSTANTIATION PHASE
# -----------------------------------------------------------
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

# Instantiated at the ACTUAL advSpawnPt
AdvAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE)

# -----------------------------------------------------------
# 5. CONSTRAINT PHASE
# -----------------------------------------------------------
require ego can see AdvAgent
terminate when (distance from ego to egoSpawnPt) > 150
terminate after 60 seconds