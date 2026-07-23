"""Scenario Description:

Under dark nighttime conditions, a high-angle view captures an ego vehicle traveling along an on-ramp on the right side, approaching a merge point with a main highway lane to its left. Two adversary vehicles are visible in the main lane, their positions marked by bright white headlights and red taillights. As the ego vehicle nears the junction, it decelerates to yield the right-of-way, allowing the two vehicles in the main lane to pass ahead. Once the adversaries have cleared the immediate merge area, the ego vehicle safely merges into the main lane, following behind them as they continue forward along the unlit road.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(10, 14)
param OPT_YIELD_DIST = Range(30, 50)       # Distance to merge point where ego begins yielding
param OPT_MERGE_CLEAR_DIST = Range(15, 25) # Distance adversaries must be past merge point before ego resumes
param OPT_BRAKE_DECEL = 0.6
param OPT_FOLLOW_GAP = Range(20, 30)

#################################
# MONITORS                      #
#################################

monitor NighttimeSetup():
    setWeather("ClearNight")
    setTimeOfDay(22)
    while True:
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoYieldAndMergeBehavior(main_lane_vehicles, yield_dist, clear_dist, target_speed):
    # Phase 1: Approach merge point on ramp
    do FollowLaneBehavior(target_speed=target_speed) until (distanceToNextJunction(self) < yield_dist)
    
    # Phase 2: Decelerate and yield until all main-lane vehicles have cleared
    take SetThrottleAction(0)
    take SetBrakeAction(globalParameters.OPT_BRAKE_DECEL)
    do WaitUntilClear(main_lane_vehicles, clear_dist)
    
    # Phase 3: Merge into main lane and follow
    do FollowLaneBehavior(target_speed=target_speed)

behavior WaitUntilClear(vehicles, clear_distance):
    while True:
        all_clear = True
        for v in vehicles:
            if (distance from self to v) < clear_distance:
                all_clear = False
                break
        if all_clear:
            break
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find a merge junction: ego on ramp (right), main lane to the left
mergeJunction = Uniform(*filter(lambda j: j.isMerge and len(j.maneuvers) >= 2, network.intersections))

# Ego maneuver: merging from ramp into main lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.MERGE, mergeJunction.maneuvers))
egoInitLane = egoManeuver.startLane
egoEndLane = egoManeuver.endLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 1: in the main lane, ahead of merge point
adv1Lane = egoEndLane
adv1SpawnPt = new OrientedPoint in adv1Lane.centerline,
    with heading adv1Lane.centerline.heading

# Adversary 2: in the main lane, further ahead of adversary 1
adv2SpawnPt = new OrientedPoint ahead of adv1SpawnPt by Range(25, 40),
    with heading adv1Lane.centerline.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

require monitor NighttimeSetup()

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoYieldAndMergeBehavior([Adv1, Adv2], globalParameters.OPT_YIELD_DIST, globalParameters.OPT_MERGE_CLEAR_DIST, globalParameters.OPT_EGO_SPEED)

Adv1 = new Car at adv1SpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

Adv2 = new Car at adv2SpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

# Ensure ego starts at a reasonable distance before the merge junction
require 40 <= (distance from egoSpawnPt to mergeJunction) <= 70

# Ensure adversaries are positioned ahead of the merge point on the main lane
require (distance from adv1SpawnPt to mergeJunction) > 5
require (distance from adv2SpawnPt to mergeJunction) > 20