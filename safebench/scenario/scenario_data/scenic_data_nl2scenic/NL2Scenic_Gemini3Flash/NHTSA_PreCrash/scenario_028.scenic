"""Scenario Description:

Vehicle is turning left at an intersection-related location, in an urban area, in daylight, 
under clear weather conditions, with a posted speed limit of 35 mph; 
and takes an evasive action (hard braking) to avoid an obstacle (a box) placed in its path.

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
SAFETY_DISTANCE = 12
BRAKE_INTENSITY = 1.0

# Weather/Daylight
param weather = 'ClearNoon'

# Vehicle Models
EGO_MODEL = 'vehicle.audi.etron'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnBehavior(trajectory):
    """
    Ego follows a left-turn trajectory at the target speed.
    If it detects an obstacle within the safety distance, it takes an evasive action (braking).
    """
    try:
        do FollowTrajectoryBehavior(target_speed=TARGET_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        # Evasive Action: Set throttle to 0 and apply full brakes
        take SetThrottleAction(0)
        take SetBrakeAction(BRAKE_INTENSITY)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for urban intersections (4-way)
urbanIntersections = filter(lambda i: i.is4Way, network.intersections)
selectedIntersec = Uniform(*urbanIntersections)

# Define a left turn maneuver at the selected intersection
leftManeuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, selectedIntersec.maneuvers)
maneuver = Uniform(*leftManeuvers)

# Define the trajectory: start lane -> intersection connection -> destination lane
egoTrajectory = [maneuver.startLane, maneuver.connectingLane, maneuver.endLane]

# Define Spawn Point for ego (back from the intersection start)
egoSpawnPt = maneuver.startLane.centerline[-1]

# Place an obstacle (a Box) in the path of the ego vehicle 
# Placing it in the connecting lane (the intersection area) to force an evasive maneuver during the turn
obstaclePos = maneuver.connectingLane.centerline.interpolate(0.6) # 60% through the turn

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Obstacle
obstacle = new Box at obstaclePos

# Spawn the Ego Vehicle
ego = new Car following roadDirection from egoSpawnPt for -Range(10, 15),
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoLeftTurnBehavior(egoTrajectory)

#################################
# CONSTRAINTS / TERMINATION     #
#################################

# Ensure the ego starts far enough back to reach speed
require (distance to selectedIntersec) >= 10

# Terminate when the ego has either stopped or passed the intersection
terminate when (distance to egoSpawnPt) > 50 or (ego.speed < 0.1 and (distance to obstacle) < SAFETY_DISTANCE + 2)