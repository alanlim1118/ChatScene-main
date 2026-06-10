"""Scenario Description:

Vehicle is turning left in an urban area, in daylight, under clear weather conditions with a posted speed limit of 35 mph; 
and encounters a pedestrian in the crosswalk at a signaled intersection.

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

# Blueprint for the ego vehicle
MODEL = 'vehicle.lincoln.mkz_2017'

# Weather and Lighting: Daylight and Clear
param weather = 'ClearNoon'

# Speed constants: 35 mph is approximately 15.65 m/s
EGO_TARGET_SPEED = 15.65 
EGO_BRAKE = 1.0

# Pedestrian constants
PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

# Safety and termination distances
SAFETY_DIST = 15.0
CRASH_DIST = 4.0
TERM_DIST = 75

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    """
    Ego vehicle follows the assigned left-turn trajectory at the target speed.
    It interrupts its movement to brake if a pedestrian is detected within a safety distance.
    """
    try:
        do FollowTrajectoryBehavior(target_speed=EGO_TARGET_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyPedestrians(self, SAFETY_DIST):
        while withinDistanceToAnyPedestrians(self, SAFETY_DIST + 3):
            take SetBrakeAction(EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Find a signalized intersection in the urban environment
signalized_intersections = filter(lambda i: i.isSignalized and (i.is3Way or i.is4Way), network.intersections)
intersection = Uniform(*signalized_intersections)

# 2. Select a left-turn maneuver from the available maneuvers at this intersection
left_maneuvers = filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*left_maneuvers)

# 3. Define the trajectory and the starting lane
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 4. Define the spawn point for the ego vehicle
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# 5. Identify the crosswalk location: usually at the entry point of the road the vehicle is turning into
exitLane = egoManeuver.endLane
pedCrosswalkPt = exitLane.centerline[0]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Spawn the pedestrian at the crosswalk
# The CrossingBehavior will dynamically adjust speed to ensure the pedestrian 
# is in the road when the ego vehicle arrives.
ped = new Pedestrian right of pedCrosswalkPt by 5,
    with behavior CrossingBehavior(ego, min_speed=PED_MIN_SPEED, threshold=PED_THRESHOLD)

# Ensure the ego vehicle starts at a reasonable distance from the intersection for the scenario to trigger
require 20 <= (distance to intersection) <= 35

# Termination condition
terminate when (distance to egoSpawnPt) > TERM_DIST