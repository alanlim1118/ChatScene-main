"""Scenario Description:

Under dark nighttime conditions, a high-angle view captures the ego vehicle traveling along an on-ramp on the right side, approaching a merge point with a main highway lane to its left. Two adversary vehicles are visible in the main lane, their positions marked by bright white headlights and red taillights. As the ego vehicle nears the junction, it decelerates to yield the right-of-way, allowing the two vehicles in the main lane to pass ahead. Once the adversaries have cleared the immediate merge area, the ego vehicle safely merges into the main lane, following behind them as they continue forward along the unlit road.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
param weather = "ClearNight"
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(10, 15)
param OPT_YIELD_DIST = Range(15, 25)
param OPT_MERGE_CLEARANCE = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior EgoBehavior(trajectory, merge_point, adv1, adv2):
    # Follow the on-ramp until near the merge point
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, trajectory) until (distance from self to merge_point) < globalParameters.OPT_YIELD_DIST
    # Yield to traffic in the main lane
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior() until (distance from adv1 to merge_point) > globalParameters.OPT_MERGE_CLEARANCE and (distance from adv2 to merge_point) > globalParameters.OPT_MERGE_CLEARANCE
    # Merge into main lane and continue along the trajectory
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Identify an on-ramp lane that feeds into a highway/main lane
rampLane = Uniform(*[lane for road in network.roads for lane in road.lanes if lane.successors and any(succ.road != lane.road for succ in lane.successors)])
mainLane = Uniform(*[succ for succ in rampLane.successors if succ.road != rampLane.road])

# Merge point at the end of the ramp
mergePoint = new OrientedPoint at rampLane.centerline.end

# Trajectory from ramp into the main lane
egoTrajectory = [rampLane, mainLane]

# Spawn points
egoSpawnPt = new OrientedPoint in rampLane.centerline

advSpawnPt1 = new OrientedPoint in mainLane.centerline
require 5 <= (distance from advSpawnPt1 to mergePoint) <= 15

advSpawnPt2 = new OrientedPoint in mainLane.centerline ahead of advSpawnPt1 by 10

#################################
# SCENARIO SPECIFICATION        #
#################################

Adv1 = new Car at advSpawnPt1,
    with heading advSpawnPt1.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

Adv2 = new Car at advSpawnPt2,
    with heading advSpawnPt2.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory, mergePoint, Adv1, Adv2)

# Requirements
require 40 <= (distance from egoSpawnPt to mergePoint) <= 60