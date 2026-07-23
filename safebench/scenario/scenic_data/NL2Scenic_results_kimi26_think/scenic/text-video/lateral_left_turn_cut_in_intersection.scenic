"""Scenario Description:

The ego vehicle travels forward through a wide urban intersection during the day with a green traffic light visible overhead. A black SUV abruptly cuts into the ego vehicle's lane from the left, seemingly maneuvering to avoid a hazard or make a turn, forcing the ego vehicle into a sudden emergency braking maneuver. The black SUV passes directly in front of the ego vehicle, resulting in a side-swipe collision with the front right side of the ego car. Following the collision, the black SUV continues to the right side of the road, and a forklift carrying a large load is visible stationary on the sidewalk to the right near the storefronts.

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
ADV_MODEL = "vehicle.audi.etron"  # SUV

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(5, 8)
param OPT_CUTIN_DIST = Range(12, 18)
param OPT_BRAKE_DIST = Range(8, 12)

OPT_EGO_BRAKE = 1

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowTrajectoryBehavior(speed, egoTrajectory)
    interrupt when (distance from self to AdvAgent < brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(OPT_EGO_BRAKE)
        do WaitBehavior() for 5 seconds
        terminate

behavior AdvCutInBehavior(adv_speed, cutin_dist, ego_vehicle):
    # Drive in the left lane until the ego vehicle is close enough
    do FollowLaneBehavior(target_speed=adv_speed) until (distance from self to ego_vehicle <= cutin_dist)
    # Abruptly steer right across the ego lane toward the right roadside
    while True:
        take SetSteerAction(0.8), SetThrottleAction(0.4)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection and a straight maneuver for the ego vehicle
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Find a forward lane section on the approach with a forward lane to the left
egoLaneSec = None
for sec in egoInitLane.sections:
    if sec.isForward and sec._laneToLeft is not None and sec._laneToLeft.isForward:
        egoLaneSec = sec
        break

require egoLaneSec is not None

advLaneSec = egoLaneSec._laneToLeft

# Spawn points along the selected lanes
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

# Forklift placement on the right sidewalk after the intersection
endLane = egoManeuver.endLane
rightCurb = endLane.laneGroup.curb
forkliftSpot = new OrientedPoint on visible rightCurb
forkliftPos = new OrientedPoint right of forkliftSpot by 2.5

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle traveling straight through the intersection
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

# Adversarial black SUV that cuts in from the left lane
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with blueprint ADV_MODEL,
    with color (0, 0, 0),
    with behavior AdvCutInBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_CUTIN_DIST, ego)

# Stationary forklift proxy (box truck representing a forklift with a large load) on the right sidewalk
forklift = new Car at forkliftPos,
    with regionContainedIn None,
    with blueprint "vehicle.carlamotors.carlacola",
    with color (1, 0.6, 0),
    with behavior WaitBehavior()

#################################
# CONSTRAINTS                   #
#################################

require (distance from AdvAgent to intersection) < (distance from ego to intersection)
require 15 <= (distance from ego to AdvAgent) <= 25
require (distance from ego to intersection) <= 60

terminate after 15 seconds