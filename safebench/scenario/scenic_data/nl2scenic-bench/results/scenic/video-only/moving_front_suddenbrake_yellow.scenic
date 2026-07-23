"""Scenario Description:

Under overcast daylight conditions, the ego vehicle travels forward in the center lane of a wide urban road, following a black sedan towards a large intersection flanked by blue construction barriers. As a large bus overtakes in the right lane, the lead vehicle continues forward, crossing the stop line and entering the intersection area. Suddenly, the lead vehicle's brake lights illuminate as it anchors its brakes abruptly, likely in response to a changing traffic signal, despite already being committed to the intersection. This unexpected halt forces the ego vehicle into a sudden emergency braking scenario, resulting in a rear-end collision with the sedan directly ahead.

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
BUS_MODEL = "vehicle.volkswagen.t2"

param EGO_SPEED = Range(8, 12)            # Ego cruising speed (m/s)
param LEAD_SPEED = Range(9, 13)           # Lead car speed before braking
param BUS_SPEED = Range(12, 16)           # Bus overtaking speed
param BRAKE_TRIGGER_DIST = Range(3, 6)    # Distance from stop line where lead car brakes
param FOLLOW_DISTANCE = Range(15, 25)     # Initial following distance behind lead car
param BUS_LATERAL_OFFSET = Range(3.5, 4.5) # Lateral offset for bus in right lane

#################################
# MONITORS                      #
#################################

monitor OvercastWeather():
    setWeather("overcast")
    wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoFollowBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, 5):
        take SetThrottleAction(0), SetBrakeAction(1)

behavior LeadCarAbruptBrakeBehavior(stopLinePoint):
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until (distance from self to stopLinePoint) <= globalParameters.BRAKE_TRIGGER_DIST
    take SetThrottleAction(0), SetBrakeAction(1)
    while True:
        wait

behavior BusOvertakeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.BUS_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a signalized 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoConnectingLane = egoManeuver.connectingLane

# Find the stop line at the intersection entrance
stopLine = egoInitLane.stopLine if hasattr(egoInitLane, 'stopLine') else egoConnectingLane.centerline[0]

# Spawn ego in center of lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead car spawns ahead of ego in same lane
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.FOLLOW_DISTANCE,
    with heading egoSpawnPt.heading

# Bus spawns in the right lane alongside or slightly behind ego
rightLane = egoInitLane.rightLane
require rightLane is not None
busSpawnPt = new OrientedPoint in rightLane.centerline,
    offset laterally by globalParameters.BUS_LATERAL_OFFSET relative to egoSpawnPt

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoFollowBehavior()

leadCar = new Car at leadSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint LEAD_CAR_MODEL,
    with color "black",
    with behavior LeadCarAbruptBrakeBehavior(stopLine)

bus = new Car at busSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint BUS_MODEL,
    with behavior BusOvertakeBehavior()

# Ensure proper spatial relationships
require distance from ego to leadCar >= globalParameters.FOLLOW_DISTANCE * 0.8
require distance from egoSpawnPt to intersection >= 30
require distance from egoSpawnPt to intersection <= 60
require monitor OvercastWeather()

terminate after 30 seconds