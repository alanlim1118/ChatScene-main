"""Scenario Description:

In a dark urban environment at night, the ego vehicle proceeds straight through a T-junction, following a lead vehicle while maintaining a safe distance. As the scene unfolds, the headlights of an oncoming adversary vehicle are visible approaching from the opposite direction; this vehicle executes a left turn into the cross-street on the right. Simultaneously, another adversary vehicle approaches the intersection from that same right-hand cross-street, its headlights illuminating the junction. The ego vehicle must carefully navigate this scenario, monitoring the turning oncoming traffic and the vehicle entering from the right to ensure safe passage past the lead vehicle.

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

param OPT_EGO_SPEED = Range(3, 5)
param OPT_LEAD_SPEED = Range(3, 5)
param OPT_ADV_ONCOMING_SPEED = Range(3, 6)
param OPT_ADV_RIGHT_SPEED = Range(3, 6)
param OPT_FOLLOW_DIST = Range(15, 25)
param OPT_BRAKE_DIST = Range(8, 12)
param OPT_HEADLIGHT_INTENSITY = 1.0

#################################
# MONITORS                      #
#################################

monitor NightAndLights():
    setWeather(dark=True, fog=0.0, precipitation=0.0)
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(LeadVehicle, 100):
            setClosestTrafficLightStatus(LeadVehicle, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoFollowBehavior(lead_vehicle, follow_dist, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to lead_vehicle < follow_dist or withinDistanceToObjsInLane(self, brake_dist)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 2 seconds
        abort
    terminate

behavior LeadBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_LEAD_SPEED, egoTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)

behavior OncomingLeftTurnBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_ONCOMING_SPEED, oncomingTrajectory)
    terminate

behavior RightCrossBehavior():
    do WaitBehavior() until (distance from self to intersection) < 40
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_RIGHT_SPEED, rightTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-junction (3-way intersection)
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego goes straight through the T-junction
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead vehicle spawns ahead of ego in the same lane
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(20, 30),
    with heading egoSpawnPt.heading

# Oncoming adversary: opposite direction, turns left into the right cross-street
oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
oncomingInitLane = oncomingManeuver.startLane
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingInitLane.centerline

# Right cross-street adversary: enters from the right cross-street toward the intersection
# Find maneuvers starting from the end lane of the oncoming left turn (the right cross-street)
rightEntryManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.LEFT_TURN, oncomingManeuver.endLane.successorManeuvers))
rightInitLane = rightEntryManeuver.startLane
rightTrajectory = [rightInitLane, rightEntryManeuver.connectingLane, rightEntryManeuver.endLane]
rightSpawnPt = new OrientedPoint in rightInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

require monitor NightAndLights()

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoFollowBehavior(LeadVehicle, globalParameters.OPT_FOLLOW_DIST, globalParameters.OPT_BRAKE_DIST),
    with headlightsOn True,
    with headlightIntensity globalParameters.OPT_HEADLIGHT_INTENSITY

LeadVehicle = new Car at leadSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior LeadBehavior(),
    with headlightsOn True,
    with headlightIntensity globalParameters.OPT_HEADLIGHT_INTENSITY

OncomingAdv = new Car at oncomingSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior OncomingLeftTurnBehavior(),
    with headlightsOn True,
    with headlightIntensity globalParameters.OPT_HEADLIGHT_INTENSITY

RightAdv = new Car at rightSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior RightCrossBehavior(),
    with headlightsOn True,
    with headlightIntensity globalParameters.OPT_HEADLIGHT_INTENSITY

require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 40 <= (distance from oncomingSpawnPt to intersection) <= 60
require 30 <= (distance from rightSpawnPt to intersection) <= 50