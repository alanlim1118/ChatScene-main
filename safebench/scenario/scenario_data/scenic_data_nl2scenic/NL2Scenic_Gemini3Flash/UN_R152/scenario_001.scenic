"""Scenario Description:
The ego vehicle travels straight through an intersection while an oncoming vehicle is 
stationary at the intersection, waiting to perform either a left or right turn.
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

EGO_MODEL = 'vehicle.tesla.model3'
ADV_MODEL = 'vehicle.audi.tt'

param EGO_SPEED = Range(8, 12)
EGO_INIT_DIST = Range(25, 35)

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'WetNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryTurningBehavior():
    """Behavior for the adversary vehicle to remain stopped at the intersection."""
    while True:
        take SetBrakeAction(1.0)
        take SetSteerAction(0.0)

behavior EgoDriveBehavior(trajectory):
    """Ego vehicle follows its trajectory and avoids collisions."""
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for intersections that allow a straight movement for ego 
# and have an oncoming lane where a vehicle can turn.
validIntersections = []
for inter in network.intersections:
    straights = [m for m in inter.maneuvers if m.type == ManeuverType.STRAIGHT]
    for s_maneuver in straights:
        # Look for the lane group going in the opposite direction
        opp_group = s_maneuver.startLane.group.opposite
        if opp_group:
            # Look for turning maneuvers from that opposite group at this intersection
            turning_maneuvers = [m for m in inter.maneuvers 
                                if m.type in [ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN] 
                                and m.startLane.group == opp_group]
            if turning_maneuvers:
                validIntersections.append((inter, s_maneuver, turning_maneuvers))

# Select a random valid scenario configuration
inter_choice, egoManeuver, advManeuverOptions = Uniform(*validIntersections)
advManeuver = Uniform(*advManeuverOptions)

# Define trajectories
egoTraj = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTraj = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Define spawn points at the stop lines (end of the incoming lanes)
egoStopLine = egoManeuver.startLane.centerline[-1]
advStopLine = advManeuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the oncoming adversary at the stop line
adversary = new Car at advStopLine,
    facing advManeuver.startLane.heading,
    with blueprint ADV_MODEL,
    with behavior StationaryTurningBehavior()

# Spawn the ego vehicle behind the intersection to allow it to gain speed
ego = new Car following roadDirection from egoStopLine for -EGO_INIT_DIST,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior(egoTraj)

# Requirements to ensure valid spatial configuration
require (distance from ego to inter_choice) > 15
require (distance from adversary to inter_choice) < 5

# Termination condition: when ego has successfully passed the intersection
terminate when (distance from ego to egoStopLine) > 50 and (ego in egoManeuver.endLane)