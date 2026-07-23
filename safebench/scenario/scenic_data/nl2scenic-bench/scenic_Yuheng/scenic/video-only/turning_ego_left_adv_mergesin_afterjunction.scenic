"""Scenario Description:

The ego vehicle navigates a large intersection under clear daylight conditions, executing a left turn onto a multi-lane urban road lined with commercial buildings and residential structures. As the turn is completed and the vehicle straightens into the left lane behind a red sedan, a yellow taxi abruptly emerges from the right side of the junction, cutting diagonally across multiple lanes without yielding. The taxi merges directly into the ego vehicle's path, resulting in a side-impact collision that strikes the front right quarter of the ego car, causing visible dashboard vibration and bringing both vehicles to an immediate halt in the travel lane.

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
LEAD_CAR_MODEL = "vehicle.tesla.model3"
TAXI_MODEL = "vehicle.ford.mustang"

param EGO_SPEED = Range(6, 9)
param LEAD_SPEED = Range(5, 8)
param TAXI_SPEED = Range(8, 12)

param SAFETY_DIST = Range(8, 15)
CRASH_DIST = 4
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
        terminate

behavior LeadCarBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)

behavior TaxiCutInBehavior(trajectory, trigger_point):
    # Wait until ego has mostly completed the turn before cutting in
    while distance from ego to trigger_point > 10:
        wait
    do FollowTrajectoryBehavior(target_speed=globalParameters.TAXI_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection for the left turn scenario
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego performs a left turn
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoEndLane = egoManeuver.endLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoEndLane]

# Spawn ego in the incoming lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead car (red sedan) is ahead of ego in the same end lane after the turn
leadSpawnOffset = Range(15, 25)
leadSpawnPt = new OrientedPoint on egoEndLane.centerline,
    ahead of egoManeuver.connectingLane.centerline.end by leadSpawnOffset

# Taxi comes from the right side of the junction and cuts diagonally across
# The right-side incoming lane relative to ego's left turn
rightIncomingLanes = filter(lambda l: 
    l is not egoInitLane and 
    l is not egoManeuver.reverseManeuvers[0].startLane if egoManeuver.reverseManeuvers else True,
    intersection.incomingLanes)
taxiInitLane = Uniform(*rightIncomingLanes)

# Taxi goes straight through the intersection, crossing ego's path
taxiManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, taxiInitLane.maneuvers))
taxiTrajectory = [taxiInitLane, taxiManeuver.connectingLane, taxiManeuver.endLane]
taxiSpawnPt = new OrientedPoint in taxiInitLane.centerline

# Trigger point for taxi cut-in: near the end of ego's connecting lane
taxiTriggerPt = egoManeuver.connectingLane.centerline.end

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with regionContainedIn None

leadCar = new Car at leadSpawnPt,
    with blueprint LEAD_CAR_MODEL,
    with color (0.8, 0.1, 0.1),  # Red sedan
    with behavior LeadCarBehavior(egoTrajectory),
    with regionContainedIn None

taxi = new Car at taxiSpawnPt,
    with blueprint TAXI_MODEL,
    with color (0.95, 0.85, 0.1),  # Yellow taxi
    with behavior TaxiCutInBehavior(taxiTrajectory, taxiTriggerPt),
    with regionContainedIn None

# Ensure ego starts at reasonable distance from intersection
require 20 <= (distance from egoSpawnPt to intersection) <= 35

# Ensure taxi starts at reasonable distance from intersection
require 15 <= (distance from taxiSpawnPt to intersection) <= 30

# Terminate simulation after sufficient distance traveled
terminate when (distance from ego to egoSpawnPt) > TERM_DIST