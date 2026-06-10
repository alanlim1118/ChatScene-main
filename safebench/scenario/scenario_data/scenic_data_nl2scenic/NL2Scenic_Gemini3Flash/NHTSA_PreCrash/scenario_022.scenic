"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
at an intersection-related location with a posted speed limit of 35 mph; 
and closes in on a stopped lead vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05' # Town05 is a squared-grid town suitable for urban intersection scenarios
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 35 mph is approximately 15.64 meters per second
SPEED_LIMIT_MS = 15.64
EGO_SPEED = 15.64

# Distance constants
LEAD_CAR_DIST_FROM_INTERSECTION = Uniform(5, 10)
INITIAL_GAP = Uniform(20, 30)

# Weather setup: clear noon for daylight and clear conditions
param weather = 'ClearNoon'

# Blueprint for vehicles
CAR_MODEL = 'vehicle.lincoln.mkz_2017'

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior():
    """Behavior for the lead vehicle to remain stopped."""
    while True:
        take SetBrakeAction(1.0)

behavior EgoDrivingBehavior(trajectory, target_speed):
    """Behavior for the ego vehicle to follow the lane towards the stopped car."""
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Filter for an urban intersection (Town05 has many 4-way intersections)
intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
inter = Uniform(*intersections)

# 2. Find a lane leading into the intersection that allows a straight maneuver
startLane = Uniform(*inter.incomingLanes)
straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, startLane.maneuvers)

# Ensure we have a straight maneuver available
require (len(list(straight_maneuvers)) > 0)
m = Uniform(*straight_maneuvers)

# 3. Define the trajectory for the ego vehicle
ego_trajectory = [m.startLane, m.connectingLane, m.endLane]

# 4. Calculate spawn points on the incoming lane
# The lead car is stopped just before the intersection
lead_spawn_pt = m.startLane.centerline[-1] # End of the lane (intersection entry)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the lead vehicle (stopped)
lead_car = new Car following roadDirection from lead_spawn_pt for -LEAD_CAR_DIST_FROM_INTERSECTION,
    with blueprint CAR_MODEL,
    with behavior LeadVehicleBehavior()

# Spawn the ego vehicle (closing in)
ego = new Car behind lead_car by INITIAL_GAP,
    with rolename 'hero',
    with blueprint CAR_MODEL,
    with behavior EgoDrivingBehavior(ego_trajectory, EGO_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure the ego vehicle is actually on the road and heading towards the intersection
require ego.lane == m.startLane
require (distance to lead_car) < 40

# Terminate when ego gets very close or a certain time passes
terminate when (distance to lead_car) < 5 or simulation().currentTime > 20