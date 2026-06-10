"""Scenario Description:

Vehicle A (ego) and vehicle B (adversary) are both heading in the same direction on a multi-lane road. 
Vehicle A is in an inner lane, while Vehicle B is in the curb lane (to the right of A). 
As they approach an intersection, Vehicle B attempts to make an illegal left turn from the curb lane, 
cutting across the path of Vehicle A. Vehicle A then strikes Vehicle B on the driver's side (left side).

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

# Blueprint models for the vehicles
EGO_MODEL = "vehicle.audi.tt"
ADV_MODEL = "vehicle.tesla.model3"

# Speed and distance parameters
param EGO_SPEED = Range(10, 15)
param ADV_SPEED = Range(8, 12)

# Distance ranges from the intersection for spawning
EGO_INIT_DIST = Range(20, 30)
ADV_INIT_DIST = Range(15, 25)

# Weather options
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
    """Ego vehicle drives straight through the intersection at a target speed."""
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

behavior AdversaryBehavior(speed, trajectory):
    """Adversary vehicle performs an illegal turn across the ego's path."""
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for 4-way intersections that support multi-lane maneuvers
intersections = filter(lambda i: i.is4Way, network.intersections)

# We need a road that has at least two forward lanes
def is_suitable_intersection(i):
    # Check if there's a straight maneuver where a lane exists to the right (curb side)
    for m in i.maneuvers:
        if m.type is ManeuverType.STRAIGHT:
            # Check if start lane has a lane to the right in the same lane section
            section = m.startLane.sections[0]
            if section.laneToRight is not None:
                return True
    return False

suitable_intersections = filter(is_suitable_intersection, intersections)
intersection = Uniform(*suitable_intersections)

# Select the ego's straight maneuver from an inner lane (one that has a lane to its right)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and 
                             m.intersection == intersection and 
                             m.startLane.sections[0].laneToRight is not None, 
                             intersection.maneuvers))

egoInitLane = egoManeuver.startLane
# The curb lane is to the right of the ego's lane
advInitLane = egoInitLane.sections[0].laneToRight.lane

# Define trajectories
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# For the illegal turn, we find a legal left turn maneuver from this road 
# and use its connecting and end lanes, but start from the curb lane.
legalLeftManeuvers = filter(lambda m: m.type is ManeuverType.LEFT_TURN and 
                            m.startLane.road == egoInitLane.road and 
                            m.intersection == intersection, 
                            intersection.maneuvers)
targetTurnManeuver = Uniform(*legalLeftManeuvers)

advTrajectory = [advInitLane, targetTurnManeuver.connectingLane, targetTurnManeuver.endLane]

# Define spawn points relative to the intersection
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED, advTrajectory)

# Ensure they are placed at specific distances from the intersection to facilitate the collision
require globalParameters.EGO_INIT_DIST.low <= (distance to intersection) <= globalParameters.EGO_INIT_DIST.high
require globalParameters.ADV_INIT_DIST.low <= (distance from adversary to intersection) <= globalParameters.ADV_INIT_DIST.high

# Ensure the adversary is slightly ahead or alongside the ego to turn across it
require 0 <= (relative heading of adversary from ego) <= 10
require (distance from ego to intersection) > (distance from adversary to intersection)

# Terminate after a reasonable simulation time
terminate when (distance to egoSpawnPt) > 80