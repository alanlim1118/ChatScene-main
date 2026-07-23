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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(8, 12)
param OPT_CIRC_SPEED = Range(5, 8)

param OPT_CUT_IN_DIST = Range(3, 6)
param OPT_COLLISION_AVOID_DIST = Range(6, 10)

TERM_DIST = 100

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_COLLISION_AVOID_DIST):
        take SetBrakeAction(1.0)
        wait

behavior AdvBehavior(adv_trajectory, ego_trajectory, intersection_obj):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=adv_trajectory)
    interrupt when (distance from self to intersection_obj) < globalParameters.OPT_CUT_IN_DIST:
        # Immediate rightward lane change / cut-in across ego's path
        take SetSteerAction(0.5)
        take SetThrottleAction(0.4)
        do WaitBehavior() for 1.5 seconds
        take SetSteerAction(0.0)
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=ego_trajectory)

behavior CircBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_CIRC_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection representing the urban roundabout
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: straight-through maneuver in the right approach lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary: left adjacent lane on the same approach
require egoInitLane.leftLane is not None
advInitLane = egoInitLane.leftLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Circulating vehicle: already in the junction ahead on ego's connecting lane
circStart = new OrientedPoint at egoManeuver.connectingLane.centerline.start,
    with heading egoManeuver.connectingLane.centerline.start.heading
circSpawnPt = new OrientedPoint ahead of circStart by Range(10, 20)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint EGO_MODEL,
    with behavior AdvBehavior(advTrajectory, [egoManeuver.connectingLane, egoManeuver.endLane], intersection)

circulating = new Car at circSpawnPt,
    with blueprint EGO_MODEL,
    with behavior CircBehavior([egoManeuver.connectingLane, egoManeuver.endLane])

# Synchronize arrival at the roundabout entrance
require 20 <= (distance from egoSpawnPt to intersection) <= 30
require 20 <= (distance from advSpawnPt to intersection) <= 30

terminate when (distance to egoSpawnPt) > TERM_DIST