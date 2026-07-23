"""Scenario Description:

The ego vehicle proceeds along a snow-covered urban street during the day, with commercial buildings and high-rise apartments visible on either side. An oncoming yellow and black taxi abruptly turns left across the ego vehicle's lane, cutting through the intersection. In response, the ego vehicle attempts to brake and swerve to avoid the taxi, but the slick, snow-packed road surface hinders traction. Consequently, the ego vehicle slides out of control and collides head-on with a white sedan traveling in the oncoming lane, which was trailing the taxi. The sequence concludes with the ego vehicle stopped directly in front of the white sedan, its hood and windshield dusted with snow from the impact.

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
TAXI_MODEL = "vehicle.dodge.charger_police"
SEDAN_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_TAXI_SPEED = Range(6, 9)
param OPT_SEDAN_SPEED = Range(7, 10)
param OPT_TAXI_TURN_TRIGGER_DIST = Range(25, 35)
param OPT_EGO_BRAKE_TRIGGER_DIST = Range(15, 22)
param OPT_SEDAN_FOLLOW_DIST = Range(12, 18)
param OPT_SLIP_FACTOR = Range(0.3, 0.5)

BRAKE_ACTION = 1.0
SWERVE_STEER = -0.4

#################################
# AGENT BEHAVIORS               #
#################################

behavior TaxiBehavior(turn_trigger_dist):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_TAXI_SPEED)
    interrupt when distance from self to ego < turn_trigger_dist:
        take SetThrottleAction(0.3), SetSteerAction(0.8), SetBrakeAction(0.1)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_TAXI_SPEED) for 3 seconds
        take SetThrottleAction(0), SetBrakeAction(0.5)

behavior EgoSnowBehavior(brake_trigger_dist, slip_factor):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when distance from self to TaxiAgent < brake_trigger_dist:
        take SetThrottleAction(0), SetBrakeAction(BRAKE_ACTION), SetSteerAction(SWERVE_STEER * slip_factor)
        wait
    interrupt when collision with SedanAgent:
        take SetThrottleAction(0), SetBrakeAction(BRAKE_ACTION)
        wait

behavior SedanFollowBehavior(leader, follow_dist):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_SEDAN_SPEED)
    interrupt when distance from self to leader < follow_dist:
        take SetThrottleAction(0), SetBrakeAction(0.6)
        wait
    interrupt when collision with ego:
        take SetThrottleAction(0), SetBrakeAction(BRAKE_ACTION)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find an intersection suitable for the taxi's left turn across ego's path
intersections = [i for i in network.intersections if i.is4Way or i.is3Way]
intersection = Uniform(*intersections)

# Ego approaches intersection on one road
egoManeuver = Uniform(*[m for m in intersection.maneuvers if m.type is ManeuverType.STRAIGHT])
egoLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Taxi is on the opposing road, will turn left across ego
taxiManeuver = Uniform(*[m for m in intersection.maneuvers 
                         if m.type is ManeuverType.LEFT_TURN 
                         and m.endLane is not egoLane
                         and m.startLane.road is not egoLane.road])
taxiLane = taxiManeuver.startLane
taxiSpawnPt = new OrientedPoint in taxiLane.centerline

# Sedan follows behind the taxi in the same lane
sedanSpawnPt = new OrientedPoint following roadDirection from taxiSpawnPt for -globalParameters.OPT_SEDAN_FOLLOW_DIST

require distance from egoSpawnPt to intersection > 40
require distance from taxiSpawnPt to intersection > 30

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set weather to snowy daytime conditions
param weather = Weather(snow=0.8, cloudiness=0.6, sunAltitude=45)

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EgoSnowBehavior(globalParameters.OPT_EGO_BRAKE_TRIGGER_DIST, globalParameters.OPT_SLIP_FACTOR)

TaxiAgent = new Car at taxiSpawnPt,
    with blueprint TAXI_MODEL,
    with color (255, 200, 0),
    with regionContainedIn None,
    with behavior TaxiBehavior(globalParameters.OPT_TAXI_TURN_TRIGGER_DIST)

SedanAgent = new Car at sedanSpawnPt,
    with blueprint SEDAN_MODEL,
    with color (255, 255, 255),
    with regionContainedIn None,
    with behavior SedanFollowBehavior(TaxiAgent, globalParameters.OPT_SEDAN_FOLLOW_DIST)

terminate after 45 seconds