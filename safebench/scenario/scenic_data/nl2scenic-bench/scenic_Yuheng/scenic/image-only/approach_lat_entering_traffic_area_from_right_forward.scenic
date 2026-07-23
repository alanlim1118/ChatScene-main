"""Scenario Description:

The ego vehicle is traveling straight in the right lane of a two-lane road. A blue adversary vehicle in the left lane performs a lateral lane change into the ego's lane from the front-left, while simultaneously a pink adversary vehicle merges into the right lane from the rear-right side. This creates a converging conflict where both adversaries enter the ego's path, requiring the ego to react to avoid collision.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(8, 12)
param ADV_LANE_CHANGE_SPEED = Range(6, 10)
param ADV_MERGE_SPEED = Range(5, 9)

param SAFETY_DIST = Range(10, 20)
CRASH_DIST = 4
TERM_DIST = 100

EGO_INIT_DIST = [30, 50]
ADV_FRONT_DIST = [25, 40]
ADV_REAR_DIST = [20, 35]

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior LaneChangeBehavior(target_speed, target_lane):
    do FollowLaneBehavior(target_speed=target_speed) until (self.lane is target_lane)
    do FollowLaneBehavior(target_speed=target_speed)

behavior MergeIntoLaneBehavior(target_speed, target_lane):
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to target_lane.centerline < 2)
    do FollowLaneBehavior(target_speed=target_speed)

behavior WaitBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment with at least 2 lanes
road = Uniform(*filter(lambda r: len(r.lanes) >= 2 and r.isRoad, network.roads))
rightLane = Uniform(*filter(lambda l: l.leftEdge is not None, road.lanes))
leftLane = rightLane.leftNeighbor

# Ego spawns in the right lane
egoSpawnPt = new OrientedPoint in rightLane.centerline

# Blue adversary spawns ahead in the left lane (will change into right lane)
blueAdvSpawnPt = new OrientedPoint in leftLane.centerline,
    ahead of egoSpawnPt by Range(ADV_FRONT_DIST[0], ADV_FRONT_DIST[1])

# Pink adversary spawns behind in the right lane or shoulder area, merging forward
pinkAdvBasePt = new OrientedPoint in rightLane.centerline,
    behind egoSpawnPt by Range(ADV_REAR_DIST[0], ADV_REAR_DIST[1])
pinkAdvSpawnPt = new OrientedPoint offset from pinkAdvBasePt by (Range(2, 4), 0),
    with heading pinkAdvBasePt.heading + Range(-15 deg, 15 deg)

# Define trajectories
egoTrajectory = [rightLane]
blueAdvTrajectory = [leftLane, rightLane]
pinkAdvTrajectory = [rightLane]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with regionContainedIn None

blueAdversary = new Car at blueAdvSpawnPt,
    with blueprint ADV_MODEL,
    with color (0, 0, 1),
    with behavior LaneChangeBehavior(globalParameters.ADV_LANE_CHANGE_SPEED, rightLane),
    with regionContainedIn None

pinkAdversary = new Car at pinkAdvSpawnPt,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior MergeIntoLaneBehavior(globalParameters.ADV_MERGE_SPEED, rightLane),
    with regionContainedIn None

require EGO_INIT_DIST[0] <= (distance from ego to road.start) <= EGO_INIT_DIST[1]
require distance from blueAdversary to ego >= ADV_FRONT_DIST[0]
require distance from pinkAdversary to ego >= ADV_REAR_DIST[0]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST