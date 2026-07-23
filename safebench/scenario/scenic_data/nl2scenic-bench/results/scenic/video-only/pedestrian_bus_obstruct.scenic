"""Scenario Description:

Under foggy daylight conditions, the ego vehicle proceeds along a narrow rural road flanked by rows of trees and green guardrails. The vehicle approaches a white bus stopped on the right side of the road and initiates an overtaking maneuver to the left. As the ego vehicle attempts to pass the stationary bus, a pedestrian suddenly steps out from the front of the bus, emerging from the ego vehicle's blind spot directly into its path. This unexpected obstruction forces the ego vehicle into a sudden emergency braking scenario, resulting in a collision with the pedestrian who was crossing the road directly in front of the bus. Following this critical moment, the footage shows the road ahead where a blue three-wheeled vehicle approaches in the oncoming lane, while other pedestrians are visible standing on the right shoulder near the guardrail.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural road map with trees and guardrails
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
BUS_MODEL = "vehicle.volkswagen.t2"
THREE_WHEEL_MODEL = "vehicle.carlamotors.carlacola"  # Placeholder for blue three-wheeled vehicle

param OPT_EGO_SPEED = Range(4, 7)
param OPT_ADV_SPEED = Range(1.5, 3.0)
param OPT_BRAKE_DIST = Range(8, 14)
param OPT_OVERTAKE_TRIGGER_DIST = Range(25, 35)
param OPT_PEDESTRIAN_TRIGGER_DIST = Range(10, 16)
param OPT_BUS_OFFSET_FROM_CURB = Range(0.5, 1.5)
param OPT_PED_OFFSET_AHEAD_OF_BUS = Range(1.0, 3.0)
param OPT_ONCOMING_DIST = Range(60, 90)
param OPT_SHOULDER_PED_OFFSET = Range(2.0, 4.0)

OPT_FOG_DENSITY = Range(0.3, 0.6)
OPT_STOP_DISTANCE = 0.5

#################################
# WEATHER                       #
#################################

param weather = Weather(
    fog_density=OPT_FOG_DENSITY,
    sun_altitude_angle=Range(30, 60),
    cloudiness=Range(20, 50)
)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoOvertakeAndBrakeBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to StoppedBus < globalParameters.OPT_OVERTAKE_TRIGGER_DIST):
        do LaneChangeBehavior(laneSectionToSwitch=ego.laneGroup._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
        try:
            do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
        interrupt when withinDistanceToAnyPedestrians(self, globalParameters.OPT_BRAKE_DIST):
            take SetThrottleAction(0), SetBrakeAction(1)
            do WaitBehavior() for 5 seconds
            terminate

behavior PedestrianEmergesFromBlindSpot(bus_ref, trigger_dist, walk_speed):
    while distance from self to ego > trigger_dist:
        wait
    take SetWalkingDirectionAction(-self.heading + 90 deg), SetWalkingSpeedAction(walk_speed)
    do CrossingBehavior(ego, walk_speed, 10) until (distance from self to ego <= 2)
    take SetWalkingSpeedAction(0)

behavior OncomingVehicleBehavior():
    do FollowLaneBehavior(target_speed=Range(3, 6))

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight rural road segment
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 2 and not s.isIntersection, network.roadSegments))
egoLane = Uniform(*filter(lambda l: l.isForward, roadSegment.lanes))
oppositeLane = roadSegment.laneToLeftOf(egoLane)

egoSpawnPt = new OrientedPoint in egoLane.centerline
busSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for Range(40, 55),
    offset laterally by globalParameters.OPT_BUS_OFFSET_FROM_CURB

pedSpawnPt = new OrientedPoint ahead of busSpawnPt by globalParameters.OPT_PED_OFFSET_AHEAD_OF_BUS,
    with heading busSpawnPt.heading

oncomingSpawnPt = new OrientedPoint in oppositeLane.centerline,
    following oppositeLane.orientation from busSpawnPt for globalParameters.OPT_ONCOMING_DIST

shoulderPedBasePt = new OrientedPoint right of busSpawnPt by globalParameters.OPT_SHOULDER_PED_OFFSET,
    with heading busSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoOvertakeAndBrakeBehavior(),
    with regionContainedIn None

# Stopped white bus on right side
StoppedBus = new Car at busSpawnPt,
    with blueprint BUS_MODEL,
    with color "255,255,255",
    with behavior WaitBehavior(),
    with regionContainedIn None

# Pedestrian emerging from blind spot in front of bus
AdvPedestrian = new Pedestrian at pedSpawnPt,
    with heading busSpawnPt.heading + 90 deg,
    with behavior PedestrianEmergesFromBlindSpot(StoppedBus, globalParameters.OPT_PEDESTRIAN_TRIGGER_DIST, globalParameters.OPT_ADV_SPEED),
    with regionContainedIn None

# Blue three-wheeled oncoming vehicle
OncomingVehicle = new Car at oncomingSpawnPt,
    with blueprint THREE_WHEEL_MODEL,
    with color "0,0,255",
    with behavior OncomingVehicleBehavior(),
    with regionContainedIn None

# Standing pedestrians on right shoulder near guardrail
StandingPed1 = new Pedestrian at shoulderPedBasePt,
    with heading shoulderPedBasePt.heading,
    with behavior WaitBehavior(),
    with regionContainedIn None

StandingPed2 = new Pedestrian ahead of shoulderPedBasePt by Range(3, 6),
    with heading shoulderPedBasePt.heading,
    with behavior WaitBehavior(),
    with regionContainedIn None

StandingPed3 = new Pedestrian behind shoulderPedBasePt by Range(2, 5),
    with heading shoulderPedBasePt.heading,
    with behavior WaitBehavior(),
    with regionContainedIn None

# Ensure sufficient distance for scenario to unfold
require distance from egoSpawnPt to busSpawnPt >= 35
require distance from busSpawnPt to oncomingSpawnPt >= 50

terminate after 45 seconds