"""Scenario Description:

The ego vehicle maintains its trajectory in a straight lane while a stationary pedestrian is 
positioned near the lane marking but outside the vehicle's direct driving path. 
The ego vehicle detects the target and executes an autonomous braking maneuver to avoid 
a potential collision/proximity risk.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Speed and distance parameters
param EGO_SPEED = Range(8, 12)          # Approx 30-40 km/h
param BRAKE_THRESHOLD = Range(15, 20)   # Distance at which AEB triggers
param SPAWN_DIST = Range(40, 50)        # Initial distance between ego and pedestrian
param LATERAL_OFFSET = Range(1.8, 2.2)  # Lateral offset to be near lane marking (lane width is ~3.5m)

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    """Behavior for the pedestrian to remain stationary."""
    while True:
        wait

behavior AEBBehavior(speed, detection_dist):
    """
    Ego behavior: Drive straight and brake when a pedestrian is detected 
    within a certain threshold distance.
    """
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyPedestrians(self, detection_dist):
        # Perform autonomous braking maneuver
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        # Hold position after braking
        while True:
            take SetBrakeAction(1.0)
            wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight lane section
lane = Uniform(*filter(lambda l: l.road.isConnector == False, network.lanes))
ego_lane_sec = Uniform(*lane.sections)

# Position the ego vehicle
ego_spawn_pt = OrientedPoint in ego_lane_sec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior AEBBehavior(globalParameters.EGO_SPEED, globalParameters.BRAKE_THRESHOLD)

# Stationary pedestrian positioned ahead and slightly to the right (near lane marking)
target_pedestrian = new Pedestrian ahead of ego by globalParameters.SPAWN_DIST,
    right of ego by globalParameters.LATERAL_OFFSET,
    with behavior StationaryBehavior()

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure the ego is on a straight road and not too close to an intersection
require (distance from ego to intersection) > 50

# Terminate the simulation once the ego has stopped and stayed still for a bit
terminate when ego.speed < 0.1 and withinDistanceToAnyPedestrians(ego, globalParameters.BRAKE_THRESHOLD + 2)
terminate after 20 seconds