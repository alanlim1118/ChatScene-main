"""Scenario Description:

The ego vehicle travels along a straight, fully marked lane at a constant speed to establish stable 
lateral control before entering a curve where it must detect and react to a stationary 
target vehicle positioned with a 0.5-meter offset from the lane center.

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

# Models
EGO_MODEL = "vehicle.audi.a2"
TARGET_MODEL = "vehicle.nissan.micra"

# Operational Parameters
param OPT_EGO_SPEED = Range(7, 10)
param OPT_BRAKE_THRESHOLD = Range(12, 18)  # Distance at which ego reacts to the stationary car
TARGET_OFFSET = 0.5                        # 0.5-meter offset from lane center
STABLE_CONTROL_DIST = 30                   # Minimum length of straight road for stabilization

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    """Behavior for the target vehicle to remain stationary."""
    while True:
        take SetBrakeAction(1.0)
        wait

behavior EgoBehavior(target_speed, brake_dist, trajectory):
    """
    Ego behavior: Follows a specific trajectory (straight then curve).
    Interrupts to brake if an object is detected in front.
    """
    try:
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        while True:
            take SetBrakeAction(1.0)
            wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find all maneuvers that involve a turn (representing a curve) 
# and have a connecting lane (the path through the intersection).
turns = [m for m in network.maneuvers if m.type in {ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN} 
         and m.connectingLane is not None 
         and m.startLane is not None]

# Select a random turn maneuver that allows for a stable approach
selectedManeuver = Uniform(*turns)
straightLane = selectedManeuver.startLane
curveLane = selectedManeuver.connectingLane
endLane = selectedManeuver.endLane

# Ensure the straight part is long enough for "stable lateral control"
require straightLane.centerline.length >= STABLE_CONTROL_DIST

# Define the trajectory for the ego vehicle
egoTrajectory = [straightLane, curveLane, endLane]

# Calculate spawn points
# Ego starts at the very beginning of the straight lane
egoSpawnPt = straightLane.centerline.start

# Target is placed at the beginning of the curve (connecting lane)
# We interpolate a short distance (e.g., 4 meters) into the curve
targetCenterPt = curveLane.centerline.interpolate(4)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Stationary Target Vehicle ---
# Positioned with a 0.5m offset (left) from the lane center
target_vehicle = new Car at targetCenterPt offset by (0, TARGET_OFFSET),
    with blueprint TARGET_MODEL,
    with behavior StationaryBehavior()

# --- Ego Vehicle ---
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED, 
        globalParameters.OPT_BRAKE_THRESHOLD, 
        egoTrajectory
    )

#################################
# ADDITIONAL CONSTRAINTS        #
#################################

# Ensure the ego has enough distance to the target at the start
require (distance from ego to target_vehicle) > 25

# Optional termination: stop scenario after ego has reacted and stopped
terminate when ego.speed < 0.1 and (distance from ego to target_vehicle) < globalParameters.OPT_BRAKE_THRESHOLD + 2