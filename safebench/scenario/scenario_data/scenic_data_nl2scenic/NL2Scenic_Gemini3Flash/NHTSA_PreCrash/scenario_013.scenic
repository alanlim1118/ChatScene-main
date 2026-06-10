"""Scenario Description:
Vehicle is turning left at an intersection in an urban area, in daylight, under clear weather conditions, 
with a posted speed limit of 35 mph (approx 15.6 m/s); and then cuts across the path of another 
vehicle initially traveling in the same direction.
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

# 35 mph is approximately 15.65 m/s
TARGET_SPEED = 15.65

# Weather and Time of Day
WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Models
CAR_MODEL = 'vehicle.lincoln.mkz_2017'

# Spawn distances
EGO_INIT_DIST = [15, 25]
ADV_INIT_DIST = [15, 25]

#################################
# AGENT BEHAVIORS               #
#################################

behavior TurnLeftBehavior(trajectory):
    """Behavior for the ego vehicle turning left at speed."""
    do FollowTrajectoryBehavior(target_speed=TARGET_SPEED, trajectory=trajectory)

behavior DriveStraightBehavior(trajectory):
    """Behavior for the adversary vehicle going straight through the intersection."""
    do FollowTrajectoryBehavior(target_speed=TARGET_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for a 4-way intersection in an urban area
intersections = filter(lambda i: i.is4Way, network.intersections)
inter = Uniform(*intersections)

# To simulate "cutting across", we find a lane where a vehicle turns left 
# from a lane that is to the right of another lane traveling in the same direction.
# This forces the turning vehicle to cross the straight-going vehicle's path.

possible_maneuvers = []
for m in inter.maneuvers:
    if m.type == ManeuverType.LEFT_TURN:
        lane = m.startLane
        # Check if there is an adjacent lane to the left in the same direction
        if lane.sections[0].laneToLeft:
            left_lane_sec = lane.sections[0].laneToLeft
            left_lane = left_lane_sec.lane
            # Ensure the left lane has a straight maneuver to be "cut across"
            straight_m = filter(lambda m2: m2.type == ManeuverType.STRAIGHT, left_lane.maneuvers)
            if straight_m:
                possible_maneuvers.append((m, list(straight_m)[0]))

# Select a pair of maneuvers that satisfy the "cut across" logic
egoManeuver, advManeuver = Uniform(*possible_maneuvers)

egoInitLane = egoManeuver.startLane
advInitLane = advManeuver.startLane

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Define spawn points on the centerlines of the respective lanes
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# The ego vehicle starts in the right-side lane and turns left
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint CAR_MODEL,
    with behavior TurnLeftBehavior(egoTrajectory)

# The adversary starts in the left-side lane (initially same direction) and goes straight
adversary = new Car at advSpawnPt,
    with blueprint CAR_MODEL,
    with behavior DriveStraightBehavior(advTrajectory)

#################################
# REQUIREMENTS                  #
#################################

# Ensure vehicles start at a reasonable distance from the intersection to gain speed
require EGO_INIT_DIST[0] <= (distance from ego to inter) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to inter) <= ADV_INIT_DIST[1]

# Ensure they are relatively close to each other to create the "cut across" conflict
require (distance from ego to adversary) < 8

# Terminate when the ego vehicle has finished the maneuver
terminate when (distance from ego to egoSpawnPt) > 50