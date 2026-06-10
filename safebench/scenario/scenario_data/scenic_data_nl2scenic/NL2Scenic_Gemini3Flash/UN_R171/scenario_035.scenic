"""Scenario Description:

The ego vehicle approaches an intersection in a straight line and initiates a turn across the path 
of an oncoming passenger car or motorcycle traveling at up to 60 km/h. 
The ego vehicle must detect the oncoming threat and intervene (brake) to prevent a collision.

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

# Models
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_CAR_MODELS = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.bmw.grandtourer', 'vehicle.toyota.prius']
ADV_BIKE_MODELS = ['vehicle.kawasaki.ninja', 'vehicle.yamaha.yzf']
ADV_MODEL = Uniform(*(ADV_CAR_MODELS + ADV_BIKE_MODELS))

# Speeds & Distances
param EGO_TARGET_SPEED = Range(5, 8)
# 60 km/h is approx 16.6 m/s
param ADV_TARGET_SPEED = Range(14, 16.6)

param SAFETY_DISTANCE = Range(12, 18)
param INITIAL_INTERSECTION_DIST = Range(20, 30)

#################################
# MONITORS                      #
#################################

monitor TrafficLightMonitor:
    # Ensure traffic lights don't impede the test scenario
    freezeTrafficLights()
    setAllIntersectionTrafficLightStatus(None, "green")
    while True:
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        take SetBrakeAction(1.0)
        wait

behavior EgoTurnAvoidanceBehavior(trajectory):
    try:
        # Proceed with the turn
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_TARGET_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DISTANCE):
        # Intervention: Detect threat and brake
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection for the scenario
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Filter for a left turn maneuver (crossing the path of oncoming traffic)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Find the oncoming straight maneuver (coming from the opposite side)
# We look for a maneuver at the same intersection that is STRAIGHT 
# and whose start lane faces the ego (relative heading ~ 180 deg)
oncomingManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT and 
    abs(relative heading of m.startLane.centerline.heading from egoInitLane.centerline.heading) > 150 deg, 
    intersection.maneuvers)

advManeuver = Uniform(*oncomingManeuvers)
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Define spawn points
egoSpawnPt = OrientedPoint in egoInitLane.centerline
advSpawnPt = OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoTurnAvoidanceBehavior(egoTrajectory)

adversary = new Vehicle at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_TARGET_SPEED, trajectory=advTrajectory)

# Requirements to ensure vehicles approach the conflict point at the same time
require globalParameters.INITIAL_INTERSECTION_DIST[0] <= (distance to intersection) <= globalParameters.INITIAL_INTERSECTION_DIST[1]
require globalParameters.INITIAL_INTERSECTION_DIST[0] <= (distance from adversary to intersection) <= globalParameters.INITIAL_INTERSECTION_DIST[1]

# Run the traffic light monitor
require monitor TrafficLightMonitor()

# Termination
terminate when (distance from ego to egoSpawnPt) > 80