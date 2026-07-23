"""Scenario Description:

Under overcast daylight conditions, the ego vehicle drives straight along a wide road bordered by large white buildings on the left and a construction site with colorful hoardings on the right. As the ego vehicle approaches an intersection, a black sedan turns right from the side road, cutting directly into the ego vehicle's path. The sedan unexpectedly decelerates immediately after merging, leaving the ego vehicle with insufficient time to react, resulting in a rear-end collision where the ego vehicle strikes the back of the black sedan and comes to a halt directly behind it.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"  # Black sedan approximation

param OPT_EGO_SPEED = Range(8, 12)        # Ego speed in m/s (~30-43 km/h)
param OPT_ADV_TURN_SPEED = Range(4, 6)    # Speed during turn maneuver
param OPT_ADV_POST_MERGE_SPEED = Range(1, 3)  # Sudden deceleration after merge
param OPT_BRAKE_DISTANCE = Range(3, 6)    # Distance at which ego attempts to brake
param OPT_MERGE_TRIGGER_DIST = Range(5, 10)  # Distance from intersection when adv completes turn
param OPT_DECEL_DELAY = Range(0.2, 0.8)   # Seconds after merge before sudden decel

CONST_RIGHT_TURN_DEG = -90 deg
CONST_TOL_DEG = 25 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_TURN_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_TURN_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort
    terminate

behavior AdvRightTurnBehavior():
    """
    Adversarial behavior: approach intersection from side road,
    execute right turn into ego's lane, then suddenly decelerate.
    """
    # Phase 1: Approach intersection on side road
    do FollowTrajectoryBehavior(trajectory=advApproachTraj, target_speed=globalParameters.OPT_ADV_TURN_SPEED) \
        until (distance from self to intersection <= globalParameters.OPT_MERGE_TRIGGER_DIST)
    
    # Phase 2: Execute right turn into ego's lane
    do FollowTrajectoryBehavior(trajectory=advTurnTraj, target_speed=globalParameters.OPT_ADV_TURN_SPEED) \
        until (self.heading is within 15 deg of egoManeuver.endLane.centerline.headingAt(self.position))
    
    # Phase 3: Brief continuation at turn speed, then sudden deceleration
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_TURN_SPEED) \
        for globalParameters.OPT_DECEL_DELAY seconds
    
    # Phase 4: Sudden hard brake / very slow crawl
    take SetThrottleAction(0)
    take SetBrakeAction(0.9)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_POST_MERGE_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a signalized 4-way intersection suitable for the scenario
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary comes from the side road to the RIGHT of ego and turns right into ego's end lane
# Filter for maneuvers that start from the right relative to ego and end in ego's end lane
def isRightTurnIntoEgoLane(m):
    return (m.type is ManeuverType.RIGHT and 
            m.endLane is egoManeuver.endLane)

advManeuver = Uniform(*filter(isRightTurnIntoEgoLane, intersection.maneuvers))
advApproachTraj = [advManeuver.startLane]
advTurnTraj = [advManeuver.connectingLane, advManeuver.endLane]
advFullTrajectory = advApproachTraj + advTurnTraj
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather: overcast daylight
param weather = 'Overcast'

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0, 0, 0),  # Black sedan
    with behavior AdvRightTurnBehavior()

# Ensure adversary is approaching from the right side relative to ego
require CONST_MIN_RIGHT_DEG < (egoDir - advDir) < CONST_MAX_RIGHT_DEG

# Ego starts 30-50m before the intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 50

# Adversary starts 15-30m before the intersection on the side road
require 15 <= (distance from advSpawnPt to intersection) <= 30

# Terminate after collision or sufficient distance past intersection
terminate when (distance from ego to intersection > 60) or \
               (distance from ego to AdvAgent < 2 and ego.speed < 0.5)