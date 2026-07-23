"""Scenario Description:

The ego vehicle proceeds along a wet, multi-lane city road under overcast conditions, where its forward visibility of a left-side junction is initially obstructed by passing oncoming traffic. A dark passenger car suddenly pulls out from this junction, crossing the double yellow lines to turn left into the ego vehicle's travel lane. The turning vehicle successfully merges ahead and accelerates, prompting the ego vehicle to maintain a safe following distance rather than colliding. As traffic flow stabilizes, a white sedan overtakes the ego vehicle in the adjacent right lane, and the drive continues straight toward a backdrop of urban high-rises, with road surface reflections and windshield droplets indicating recent rainfall and damp driving conditions.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.a2"
SEDAN_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(10, 14)
param SEDAN_SPEED = Range(13, 16)
param ONCOMING_SPEED = Range(9, 13)
param SAFETY_DIST = Range(10, 15)

#################################
# WEATHER / ENVIRONMENT         #
#################################

param weather = {
    'cloudiness': 100.0,
    'precipitation': 0.0,
    'precipitation_deposits': 80.0,
    'wetness': 100.0,
    'sun_altitude_angle': 20.0,
    'fog_density': 25.0
}

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(0.8)
    interrupt when withinDistanceToAnyObjs(self, 3):
        terminate

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-junction so that a left turn from the side road merges onto the main road
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego travels straight through the intersection along the main road
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Identify the side-road incoming lane (left-side junction, roughly perpendicular to ego)
egoHeading = egoInitLane.centerline.heading
advInitLane = Uniform(*filter(lambda l: l is not egoInitLane and abs(l.centerline.heading - egoHeading) > 60 deg and abs(l.centerline.heading - egoHeading) < 120 deg, intersection.incomingLanes))

# Adversary performs a left turn from the side road onto the main road (same direction as ego)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.startLane is advInitLane, intersection.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Oncoming traffic in the opposite main-road lane to obstruct visibility
oncomingLane = Uniform(*filter(lambda l: l is not egoInitLane and l is not advInitLane, intersection.incomingLanes))

# Spawn points
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Oncoming car placed so it passes the junction as ego approaches
oncomingSpawnPt = new OrientedPoint following oncomingLane.orientation from oncomingLane.centerline.end by -18

# White sedan spawn point: behind ego on the adjacent right lane after the intersection
endLaneStart = new OrientedPoint at egoManeuver.endLane.centerline.start
sedanCenterPt = new OrientedPoint following egoManeuver.endLane.orientation from endLaneStart by -35

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Dark adversary passenger car pulling out from the left-side junction
adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (0.05, 0.05, 0.05),
    with behavior AdversaryBehavior(advTrajectory)

# Oncoming traffic obstructing the view of the junction
oncomingCar = new Car at oncomingSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED)

# White sedan overtaking in the adjacent right lane
whiteSedan = new Car right of sedanCenterPt by 3.5,
    with heading sedanCenterPt.heading,
    with blueprint SEDAN_MODEL,
    with color (1, 1, 1),
    with behavior FollowLaneBehavior(target_speed=globalParameters.SEDAN_SPEED)

# Spatial requirements to stage the encounter
require 20 <= (distance to intersection) <= 40
require 8 <= (distance from adversary to intersection) <= 22
terminate when (distance to egoSpawnPt) > 150