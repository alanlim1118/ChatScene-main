"""Scenario Description:

A white ego vehicle travels forward along its centerline axis towards a bicyclist crossing its path from the nearside, emerging from behind an obstruction formed by two stationary vehicles parked on the right. The bicyclist travels along its trajectory, accelerating over a distance outside the field of view and covering a steady-state distance of 17.00 meters to reach the impact point positioned on the vehicle's centerline. The scenario depicts a collision where the frontal structure of the vehicle strikes the bicyclist at this intersection point without any braking action being applied, with the cyclist's path laterally offset by 1.00 meter from the parked obstruction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = 10.0  # m/s

BICYCLE_SPEED = 5.0  # m/s
STEADY_STATE_DIST = 17.00
LATERAL_OFFSET = 1.00

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Travel forward along the lane without braking
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior BicycleCrossingBehavior(trajectory):
    # Follow the crossing trajectory at constant speed (steady-state portion)
    do FollowTrajectoryBehavior(target_speed=globalParameters.BICYCLE_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road lane
lane = Uniform(*network.lanes)

# Ego spawn point on the lane centerline
egoSpawnPt = new OrientedPoint in lane.centerline

# Impact point on the vehicle's centerline, ahead of the ego
impactPoint = new OrientedPoint ahead of egoSpawnPt by 40

# Parked vehicles on the right side forming the obstruction
# Placed before the impact point, offset to the right
parkedSpot1 = new OrientedPoint ahead of egoSpawnPt by 15,
    offset by 0 @ -3.5
parkedSpot2 = new OrientedPoint ahead of egoSpawnPt by 20,
    offset by 0 @ -3.5

# Cyclist start point: on the nearside, behind the parked obstruction,
# with the trajectory laterally offset 1.00m from the parked vehicles.
# The cyclist travels ~17.00m in steady state to the impact point.
cyclistStartPt = new OrientedPoint ahead of egoSpawnPt by 23.6,
    offset by 0 @ -4.5

# Trajectory from cyclist start to impact point
cyclistTrajectory = [cyclistStartPt, impactPoint]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (white)
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color Color(1, 1, 1),
    with behavior EgoBehavior()

# Parked obstruction vehicles
parked1 = new Car at parkedSpot1,
    with heading egoSpawnPt.heading

parked2 = new Car at parkedSpot2,
    with heading egoSpawnPt.heading

# Bicyclist crossing from nearside
bicycle = new Bicycle at cyclistStartPt,
    with heading (heading of (impactPoint - cyclistStartPt)),
    with behavior BicycleCrossingBehavior(cyclistTrajectory),
    with regionContainedIn None

# Ensure impact point is ahead and scenario runs long enough
require (distance from ego to impactPoint) >= 35
terminate when (distance from ego to impactPoint) > 60