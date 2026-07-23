"""Scenario Description:

In a low-light urban environment at night, the ego vehicle is positioned at a four-way intersection attempting to execute a left turn. The scene is dominated by the headlights of multiple adversary vehicles approaching from the opposing arm of the intersection. These oncoming vehicles proceed straight through the junction, their lights moving across the ego vehicle's field of view as they traverse the crossing. Because this stream of traffic directly intersects the ego vehicle's intended path, the ego vehicle is forced to yield, holding its position or moving slowly to allow the adversaries to pass. The maneuver requires careful timing to wait for a sufficient gap in the oncoming traffic flow before safely completing the left turn without causing a collision.

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

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)
param EGO_YIELD_SPEED = VerifaiRange(0, 2)

ADV_INIT_DIST = [25, 45]
param ADV_SPEED = VerifaiRange(8, 12)
NUM_ADVERSARIES = Range(2, 4)

param SAFETY_DIST = VerifaiRange(12, 20)
CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnYieldBehavior(trajectory):
    """Ego attempts left turn but yields to oncoming traffic."""
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior AdversaryStraightBehavior(trajectory):
    """Adversary proceeds straight through intersection at constant speed."""
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: left turn maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries: oncoming lane going straight (conflicting with ego's left turn)
advInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers)
    ).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set nighttime / low-light conditions
param timeOfDay = 'night'
param weather = 'ClearNight'

# Create ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoLeftTurnYieldBehavior(egoTrajectory),
    with headlights True

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]

# Create multiple adversary vehicles in the oncoming lane
adversaries = []
for i in range(NUM_ADVERSARIES):
    advSpawnPt = new OrientedPoint in advInitLane.centerline
    adv = new Car at advSpawnPt,
        with blueprint MODEL,
        with behavior AdversaryStraightBehavior(advTrajectory),
        with headlights True
    adversaries.append(adv)
    
    # Ensure adversaries are spaced apart and within valid distance range
    require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]
    if i > 0:
        require (distance from advSpawnPt to adversaries[i-1].position) >= 15

terminate when (distance from ego to egoSpawnPt) > TERM_DIST