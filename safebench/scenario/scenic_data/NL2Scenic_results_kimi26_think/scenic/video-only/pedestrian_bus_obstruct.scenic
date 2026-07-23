"""Scenario Description:

Under foggy daylight conditions, the ego vehicle proceeds along a narrow rural road flanked by rows of trees and green guardrails. The vehicle approaches a white bus stopped on the right side of the road and initiates an overtaking maneuver to the left. As the ego vehicle attempts to pass the stationary bus, a pedestrian suddenly steps out from the front of the bus, emerging from the ego vehicle's blind spot directly into its path. This unexpected obstruction forces the ego vehicle into a sudden emergency braking scenario, resulting in a collision with the pedestrian who was crossing the road directly in front of the bus. Following this critical moment, the footage shows the road ahead where a blue three-wheeled vehicle approaches in the oncoming lane, while other pedestrians are visible standing on the right shoulder near the guardrail.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BUS_DISTANCE = Range(30, 45)
param OPT_EGO_LC_DISTANCE = Range(10, 15)
param OPT_PED_SPEED = Range(2, 4)
param OPT_PED_ADV_DIST = Range(5, 8)
param OPT_PED_STOP_DIST = 1.0
param OPT_BRAKE_DIST = Range(4, 7)
param OPT_ONCOMING_DIST = Range(50, 70)
param OPT_SHOULDER_OFFSET = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
    do CrossingBehavior(actor_reference, adv_speed, adv_distance) until (distance from self to stop_reference <= stop_distance)
    take SetWalkingSpeedAction(0)

behavior EgoOvertakeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to Bus < globalParameters.OPT_EGO_LC_DISTANCE)
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoLaneSec = network.laneSectionAt(egoSpawnPt)

# Bus stopped on the right side of the road ahead
busRoadPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_BUS_DISTANCE
busPos = new OrientedPoint right of busRoadPt by 2

# Pedestrian at front of bus (blind spot), slightly toward the travel lane
pedOffset = 2.5 @ 0.5
pedSpawnPt = new OrientedPoint at busPos offset along roadDirection by pedOffset

# Oncoming three-wheeled vehicle ahead in the left (oncoming) lane
oncomingRef = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ONCOMING_DIST
threeWheelSpawn = new OrientedPoint left of oncomingRef by 4

# Bystanders on the right shoulder near the guardrail
shoulderPt = new OrientedPoint right of busRoadPt by globalParameters.OPT_SHOULDER_OFFSET
shoulderPt2 = new OrientedPoint right of busRoadPt by globalParameters.OPT_SHOULDER_OFFSET + 1.5

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather: foggy daylight (configure via simulator API if parametric weather is unsupported)
# Recommended settings: cloudiness=30, fog_density=60, fog_distance=20, sun_altitude_angle=45

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoOvertakeBehavior()

# White bus stopped on the right side
Bus = new Car at busPos,
    with heading roadDirection,
    with regionContainedIn None,
    with behavior WaitBehavior()

# Pedestrian steps out from front of bus into ego's path
AdvPedestrian = new Pedestrian at pedSpawnPt,
    with heading roadDirection + 90 deg,
    with regionContainedIn None,
    with behavior CrossAndStopBehavior(
        ego,
        globalParameters.OPT_PED_SPEED,
        globalParameters.OPT_PED_ADV_DIST,
        egoInitLane.centerline,
        globalParameters.OPT_PED_STOP_DIST
    )

# Blue three-wheeled vehicle approaching in oncoming lane
ThreeWheeler = new Car at threeWheelSpawn,
    with heading roadDirection + 180 deg,
    with regionContainedIn None,
    with behavior FollowLaneBehavior(target_speed=Range(6, 9))

# Other pedestrians standing on right shoulder
Bystander1 = new Pedestrian at shoulderPt,
    with heading roadDirection,
    with regionContainedIn None,
    with behavior WaitBehavior()

Bystander2 = new Pedestrian at shoulderPt2,
    with heading roadDirection,
    with regionContainedIn None,
    with behavior WaitBehavior()

require (distance from egoSpawnPt to intersection) >= 80
terminate after 30 seconds