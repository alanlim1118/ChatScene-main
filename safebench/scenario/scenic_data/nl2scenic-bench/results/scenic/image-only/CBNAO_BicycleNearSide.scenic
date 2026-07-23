"""Scenario Description:

In this traffic scenario, a white vehicle under test travels forward along its centerline axis BB towards a bicyclist crossing its path from the nearside, emerging from behind an obstruction formed by two stationary vehicles parked on the right. The bicyclist travels along trajectory AA, accelerating over a distance C outside the field of view and covering a steady-state distance D of 17.00 meters to reach the impact point E, which is positioned on the vehicle's centerline. The scenario depicts a collision where the frontal structure of the vehicle strikes the bicyclist at this intersection point without any braking action being applied, with the cyclist's path laterally offset by 1.00 meter from the parked obstruction.

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
PARKED_CAR_MODEL = "vehicle.tesla.model3"

STEADY_STATE_DISTANCE = 17.00  # Distance D in meters
LATERAL_OFFSET = 1.00          # Cyclist path offset from parked obstruction in meters
ACCELERATION_DISTANCE = 10.0   # Distance C for acceleration phase (outside FOV)
CYCLIST_CRUISING_SPEED = 4.0   # Steady-state speed for cyclist
EGO_SPEED = 8.0                # Ego vehicle constant speed (no braking)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoNoBrakeBehavior(speed):
    """Ego drives forward at constant speed without any braking."""
    do FollowLaneBehavior(target_speed=speed)

behavior CyclistCrossingBehavior(accel_distance, cruise_speed, steady_distance):
    """Cyclist accelerates then maintains steady speed to cross ego's path."""
    # Acceleration phase (simulated as lower initial speed ramp-up)
    accel_speed = cruise_speed * 0.5
    do FollowLaneBehavior(target_speed=accel_speed) until (distance traveled >= accel_distance)
    # Steady-state crossing phase
    do FollowLaneBehavior(target_speed=cruise_speed) until (distance traveled >= accel_distance + steady_distance)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment suitable for the scenario
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 2, network.roadSegments))
egoLane = Uniform(*filter(lambda l: l._laneToLeft is not None or l._laneToRight is not None, roadSegment.lanes))

# Ego spawn point on centerline
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Impact point E is ahead of ego on the centerline by steady-state distance + some buffer
impactPoint = new OrientedPoint ahead of egoSpawnPt by (STEADY_STATE_DISTANCE + ACCELERATION_DISTANCE + 5),
    facing roadDirection

# Parked obstruction: two stationary vehicles on the right side of ego's lane
rightEdge = egoLane.rightEdge
parkedCar1Pos = new OrientedPoint following rightEdge from impactPoint for -3,
    facing roadDirection
parkedCar2Pos = new OrientedPoint following rightEdge from impactPoint for 3,
    facing roadDirection

# Cyclist spawn point: laterally offset from parked obstruction by LATERAL_OFFSET
# Position cyclist so that after traveling STEADY_STATE_DISTANCE they reach impactPoint
cyclistStartOffset = STEADY_STATE_DISTANCE + ACCELERATION_DISTANCE
cyclistSpawnBase = new OrientedPoint following rightEdge from impactPoint for -cyclistStartOffset,
    facing roadDirection
cyclistSpawnPt = new OrientedPoint at cyclistSpawnBase offset by LATERAL_OFFSET @ 90 deg,
    facing toward impactPoint

#################################
# SCENARIO SPECIFICATION        #
#################################

# White ego vehicle under test
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "255,255,255",
    with behavior EgoNoBrakeBehavior(EGO_SPEED),
    with regionContainedIn None

# Two stationary parked cars forming obstruction
parkedCar1 = new Car at parkedCar1Pos,
    with blueprint PARKED_CAR_MODEL,
    with regionContainedIn None

parkedCar2 = new Car at parkedCar2Pos,
    with blueprint PARKED_CAR_MODEL,
    with regionContainedIn None

# Bicyclist crossing from nearside (right)
bicyclist = new Bicycle at cyclistSpawnPt,
    with heading cyclistSpawnPt.heading,
    with behavior CyclistCrossingBehavior(ACCELERATION_DISTANCE, CYCLIST_CRUISING_SPEED, STEADY_STATE_DISTANCE),
    with regionContainedIn None

# Ensure ego has sufficient distance to the impact point
require distance from egoSpawnPt to impactPoint >= (STEADY_STATE_DISTANCE + ACCELERATION_DISTANCE)

# Terminate after collision or when ego passes impact point significantly
terminate when (distance from ego to impactPoint > 30) or (collision between ego and bicyclist)