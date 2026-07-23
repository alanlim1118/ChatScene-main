"""Scenario Description:

The ego vehicle travels forward on a straight, paved road in a semi-rural area with buildings on the right and open land on the left under clear skies. As the vehicle nears an intersection, a silver minivan approaches from the right side road. The van proceeds to drive straight across the intersection without yielding, cutting directly in front of the ego vehicle. This action results in a sudden side-impact collision as the van obstructs the ego vehicle's path, forcing a hazardous interaction at the junction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.volkswagen.t6"  # Silver minivan-like vehicle

param OPT_EGO_SPEED = Range(8, 12)       # Ego speed in m/s (~30-43 km/h)
param OPT_ADV_SPEED = Range(8, 14)       # Adversary speed for crossing
param OPT_BRAKE_DIST = Range(15, 25)     # Distance at which ego attempts to brake
param OPT_ADV_TRIGGER_DIST = Range(30, 50)  # Distance from intersection when adv starts crossing

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoStraightBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0), SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior CrossIntersectionNoYieldBehavior(trigger_distance):
    # Wait until ego is close enough to the intersection before proceeding
    while distance from self to ego > trigger_distance:
        wait
    # Drive straight across the intersection at constant speed without yielding
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    terminate after 10 seconds

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way or 3-way intersection suitable for the scenario
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoStartLane = egoManeuver.startLane
egoEndLane = egoManeuver.endLane
egoTrajectory = [egoStartLane, egoManeuver.connectingLane, egoEndLane]

# Place ego on the start lane centerline, approaching the intersection
egoSpawnPt = new OrientedPoint in egoStartLane.centerline

# Identify the right-side incoming lane at the intersection for the adversary
# The adversary comes from the right relative to ego's direction of travel
rightIncomingManeuvers = filter(
    lambda m: m.type is ManeuverType.STRAIGHT and 
              abs(relativeAngle(m.startLane.centerline.end.heading, egoStartLane.centerline.end.heading) + 90 deg) < 30 deg,
    intersection.maneuvers
)
advManeuver = Uniform(*rightIncomingManeuvers)
advStartLane = advManeuver.startLane

# Spawn adversary on the right side road, approaching the intersection
advSpawnPt = new OrientedPoint in advStartLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoStraightBehavior(),
    with color (0.8, 0.8, 0.8)  # Light gray/silver appearance

adversary = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CrossIntersectionNoYieldBehavior(globalParameters.OPT_ADV_TRIGGER_DIST),
    with color (0.75, 0.75, 0.78)  # Silver minivan color

# Ensure ego starts at a reasonable distance from the intersection
require 40 <= (distance from ego to intersection) <= 70

# Ensure adversary starts at a reasonable distance from the intersection on the right road
require 30 <= (distance from adversary to intersection) <= 60

# Ensure the adversary is indeed to the right of ego's path
require relativeAngle(ego.heading, heading from ego to adversary) < 0 deg

terminate after 30 seconds