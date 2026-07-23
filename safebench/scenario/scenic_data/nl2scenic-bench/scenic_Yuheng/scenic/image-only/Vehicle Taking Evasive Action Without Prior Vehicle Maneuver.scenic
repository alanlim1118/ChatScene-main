"""Scenario Description:

In an urban area during daylight with clear weather conditions, a vehicle is traveling straight on a road at a non-junction location with a posted speed limit of 35 mph. The vehicle, positioned in the right lane, approaches a black obstacle directly ahead in its path. To avoid a collision, the driver takes evasive action by swerving to the left, following a trajectory indicated by a dashed curved arrow that leads towards the roadside where a tree is situated. This maneuver appears to direct the vehicle across the lane of another car that is traveling straight in the adjacent left lane.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"
OBSTACLE_MODEL = "static.prop.streetbarrier"

# Speed limit 35 mph ≈ 15.6 m/s
param EGO_SPEED = Range(14, 16)
param ADV_SPEED = Range(12, 15)

param OBSTACLE_DISTANCE = Range(40, 60)
param SWERVE_TRIGGER_DIST = Range(20, 30)
param LANE_WIDTH = 3.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoSwerveBehavior(straight_lane, left_lane, trigger_dist, target_speed):
    """
    Ego drives straight in right lane until obstacle is within trigger_dist,
    then swerves left into adjacent lane following a curved path.
    """
    # Phase 1: Drive straight in right lane
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to egoObstacle <= trigger_dist)
    
    # Phase 2: Swerve left - follow the left lane centerline as evasive trajectory
    do FollowLaneBehavior(target_speed=target_speed * 0.8)

behavior AdvStraightBehavior(target_speed):
    """Adversary vehicle travels straight in the left lane."""
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-junction straight road segment
roadSegment = Uniform(*filter(
    lambda s: len(s.lanes) >= 2 and not any(i for i in network.intersections if s in i.roads),
    network.roads
))

# Right lane for ego, left lane for adversary
rightLane = Uniform(*filter(lambda l: l.isForward, roadSegment.lanes))
leftLane = Uniform(*filter(lambda l: l.isForward and l is not rightLane and 
                           (l.leftNeighbor is rightLane or l.rightNeighbor is rightLane),
                           roadSegment.lanes))

# Ego spawn point in right lane
egoSpawnPt = new OrientedPoint in rightLane.centerline

# Obstacle placed ahead of ego in right lane
egoObstacle = new Object at egoSpawnPt offset by (0, globalParameters.OBSTACLE_DISTANCE),
    with blueprint OBSTACLE_MODEL,
    with regionContainedIn rightLane,
    with heading egoSpawnPt.heading

# Adversary spawn point in left lane, slightly behind or alongside ego
advOffset = Range(-10, 10)
advSpawnPt = new OrientedPoint at egoSpawnPt offset by (-globalParameters.LANE_WIDTH, advOffset),
    with heading egoSpawnPt.heading

# Tree near roadside on the left side (visual reference for swerve destination)
treeRegion = leftLane.leftEdge.offsetBy(2)
treeSpawnPt = new OrientedPoint in treeRegion,
    with heading leftLane.centerline.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set weather and time of day
param weather = "Clear"
param timeOfDay = 12  # Noon / daylight

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoSwerveBehavior(rightLane, leftLane, globalParameters.SWERVE_TRIGGER_DIST, globalParameters.EGO_SPEED),
    with regionContainedIn None

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvStraightBehavior(globalParameters.ADV_SPEED),
    with regionContainedIn None

tree = new Object at treeSpawnPt,
    with blueprint "static.prop.tree_large",
    with regionContainedIn None

# Ensure valid spatial configuration
require rightLane is not leftLane
require distance from egoSpawnPt to egoObstacle >= 30
require distance from advSpawnPt to egoSpawnPt <= 20

# Terminate after sufficient distance traveled
terminate when (distance from ego to egoSpawnPt) > 120