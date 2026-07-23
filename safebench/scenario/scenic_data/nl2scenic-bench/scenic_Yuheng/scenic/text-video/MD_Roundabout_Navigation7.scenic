"""Scenario Description:

Under dark nighttime conditions, the ego vehicle proceeds straight toward and enters an urban multi-lane roundabout junction. As the ego vehicle approaches the entrance, a vehicle on the left simultaneously enters the roundabout while another vehicle is already circulating ahead within the junction. The simultaneously entering vehicle on the left immediately attempts a rightward lane change, cutting into the ego vehicle's intended path. The scene is characterized by extremely low visibility, with only the headlights and taillights of the involved vehicles clearly illuminating the dark roadway and roundabout geometry as the potential lateral conflict unfolds.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(5, 8)
param ADV_ENTER_SPEED = Range(5, 8)
param CIRCULATING_SPEED = Range(6, 9)
param LANE_CHANGE_DIST = Range(15, 25)
param BRAKE_DIST = Range(8, 12)
param TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior CirculatingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.CIRCULATING_SPEED)

behavior CutInBehavior(ego_ref, trigger_dist):
    do FollowLaneBehavior(target_speed=globalParameters.ADV_ENTER_SPEED) until (distance from self to ego_ref <= trigger_dist)
    do LaneChangeBehavior(direction='right', target_speed=globalParameters.ADV_ENTER_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_ENTER_SPEED)

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (withinDistanceToAnyObjs(self, globalParameters.BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a roundabout intersection
roundabout = Uniform(*filter(lambda i: i.isRoundabout, network.intersections))

# Ego enters the roundabout going straight through it
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, roundabout.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary entering from the left incoming lane of the roundabout
leftIncomingLanes = filter(lambda l: l is not egoInitLane, roundabout.incomingLanes)
advEnterLane = Uniform(*leftIncomingLanes)
advEnterManeuver = Uniform(*filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN), advEnterLane.maneuvers))
advEnterTrajectory = [advEnterLane, advEnterManeuver.connectingLane, advEnterManeuver.endLane]
advEnterSpawnPt = new OrientedPoint in advEnterLane.centerline

# Circulating vehicle already inside the roundabout
circulatingLanes = roundabout.internalLanes
circulatingLane = Uniform(*circulatingLanes)
circulatingSpawnPt = new OrientedPoint in circulatingLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime weather with minimal ambient light
param weather = Weather(preset='ClearNight', sunAltitude=-10, fogDensity=0.0, wetness=0.0)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(),
    with headlightsOn True

adversary = new Car at advEnterSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CutInBehavior(ego, globalParameters.LANE_CHANGE_DIST),
    with headlightsOn True

circulator = new Car at circulatingSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CirculatingBehavior(),
    with headlightsOn True

require 30 <= (distance from egoSpawnPt to roundabout) <= 45
require 25 <= (distance from advEnterSpawnPt to roundabout) <= 40
require (distance from circulator to roundabout) <= 10
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST