"""Scenario Description:

This top-down aerial view captures a traffic scenario on a road network flanked by a residential area with houses and trees on the left and a dense forest on the right. The ego vehicle, represented by a green car, travels straight forward in the right lane of the main vertical roadway, following a red car at a distance. Simultaneously, a blue adversarial vehicle navigates a curved entrance ramp merging from the right side, with a visible blue trajectory line indicating its path as it prepares to merge into the main lane ahead of the ego vehicle. The environment features clear lane markings, streetlights, and shadows cast by the surrounding vegetation and buildings, suggesting a daytime setting.

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
LEADING_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEADING_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(10, 14)
param OPT_FOLLOW_DISTANCE = Range(25, 40)
param OPT_BRAKE_DISTANCE = Range(10, 18)
param OPT_ADV_MERGE_DISTANCE = Range(50, 70)  # Distance at which adv begins merge maneuver

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=globalParameters.OPT_BRAKE_DISTANCE):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

behavior LeadingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

behavior AdvMergeBehavior():
    # Navigate the curved entrance ramp and merge into the main lane
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable intersection or road segment with a merging ramp
# We look for maneuvers that represent straight travel on the main road
mainManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, network.maneuvers))
mainLane = mainManeuver.startLane
egoTrajectoryLine = mainLane.centerline + mainManeuver.connectingLane.centerline + mainManeuver.endLane.centerline

# Spawn point for ego in the right lane of the main vertical roadway
egoSpawnPt = new OrientedPoint in mainLane.centerline

# Leading car spawn point ahead of ego
leadingSpawnPt = new OrientedPoint following mainLane.orientation from egoSpawnPt for globalParameters.OPT_FOLLOW_DISTANCE

# Find a conflicting/merging maneuver from the right side (entrance ramp)
mergeManeuvers = filter(lambda m: m.type is not ManeuverType.STRAIGHT, mainManeuver.conflictingManeuvers)
advManeuver = Uniform(*mergeManeuvers) if mergeManeuvers else mainManeuver
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0, 255, 0),  # Green car
    with behavior EgoBehavior()

LeadingCar = new Car at leadingSpawnPt,
    with regionContainedIn None,
    with blueprint LEADING_MODEL,
    with color (255, 0, 0),  # Red car
    with behavior LeadingBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0, 0, 255),  # Blue adversarial vehicle
    with behavior AdvMergeBehavior()

# Ensure the adversarial vehicle starts on the ramp at a reasonable distance
require 30 <= (distance from advSpawnPt to egoSpawnPt) <= 80
# Ensure leading car is properly ahead of ego
require 20 <= (distance from egoSpawnPt to leadingSpawnPt) <= 50
# Terminate after ego has passed the merge zone
terminate when distance from ego to egoSpawnPt > 120