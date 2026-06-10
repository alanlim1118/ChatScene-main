"""Scenario Description:

Vehicle is turning left in an urban area, in daylight, under clear weather conditions, 
at a signalized intersection with a posted speed limit of 35 mph; 
and cuts across the path of another vehicle straight crossing from an opposite direction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 35 mph is approximately 15.6 m/s
TARGET_SPEED = 15.6
TURN_SPEED = 8

# Weather and Time
param weather = 'ClearNoon'

# Spawn distances
EGO_DIST = Uniform(20, 25)
ADV_DIST = Uniform(20, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnBehavior(trajectory):
    # Ego vehicle follows the turn trajectory without yielding to simulate "cutting across"
    do FollowTrajectoryBehavior(target_speed=TURN_SPEED, trajectory=trajectory)

behavior AdversaryStraightBehavior(trajectory):
    # Adversary vehicle drives straight through the intersection
    do FollowTrajectoryBehavior(target_speed=TARGET_SPEED, trajectory=trajectory)

#################################
# MONITORS                      #
#################################

monitor TrafficLightMonitor(intersection):
    # Ensure both vehicles have a green light to create the conflict
    freezeTrafficLights()
    setAllIntersectionTrafficLightStatus(intersection, "green")
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for signalized 4-way or 3-way intersections in an urban-like area
intersections = filter(lambda i: i.isSignalized and (i.is4Way or i.is3Way), network.intersections)
intersec = Uniform(*intersections)

# 1. Pick a straight maneuver for the adversary
adv_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersec.maneuvers)
adv_maneuver = Uniform(*adv_maneuvers)

# 2. Pick a left turn maneuver for the ego that conflicts with the adversary
# and starts from a lane that is roughly opposite to the adversary's start lane.
# Conflicting maneuvers from the opposite side are usually those that intersect the straight path.
ego_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN 
                       and m in adv_maneuver.conflictingManeuvers, intersec.maneuvers)

# Refine to ensure it comes from the "opposite" direction (approx 180 degrees difference in heading)
def is_opposite(m1, m2):
    h1 = m1.startLane.centerline.heading
    h2 = m2.startLane.centerline.heading
    diff = abs(h1 - h2)
    return 160 < diff < 200

ego_opposite_maneuvers = filter(lambda m: is_opposite(m, adv_maneuver), ego_maneuvers)
ego_maneuver = Uniform(*ego_opposite_maneuvers)

# Define Trajectories
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

# Calculate spawn points
ego_spawn_pt = ego_maneuver.startLane.centerline[-1]
adv_spawn_pt = adv_maneuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Adversary first
adversary = new Car following roadDirection from adv_spawn_pt for -ADV_DIST,
    with behavior AdversaryStraightBehavior(adv_trajectory)

# Spawn Ego
ego = new Car following roadDirection from ego_spawn_pt for -EGO_DIST,
    with rolename 'hero',
    with behavior EgoLeftTurnBehavior(ego_trajectory)

# Requirements
require monitor TrafficLightMonitor(intersec)
require 15 <= (distance from ego to intersec) <= 30
require 15 <= (distance from adversary to intersec) <= 30

# Terminate when ego completes the turn or moves far away
terminate when (distance from ego to ego_spawn_pt) > 80