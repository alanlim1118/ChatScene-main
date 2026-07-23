"""Scenario Description:

The ego vehicle proceeds along a wet, multi-lane city road under overcast conditions, where its forward visibility of a left-side junction is initially obstructed by passing oncoming traffic. A dark passenger car suddenly pulls out from this junction, crossing the double yellow lines to turn left into the ego vehicle's travel lane. The turning vehicle successfully merges ahead and accelerates, prompting the ego vehicle to maintain a safe following distance rather than colliding. As traffic flow stabilizes, a white sedan overtakes the ego vehicle in the adjacent right lane, and the drive continues straight toward a backdrop of urban high-rises, with road surface reflections and windshield droplets indicating recent rainfall and damp driving conditions.

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
ADV_MODEL = "vehicle.tesla.model3"       # Dark passenger car
OVERTAKE_MODEL = "vehicle.mercedes.coupe" # White sedan

param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(6, 9)
param OVERTAKE_SPEED = Range(12, 16)
param SAFE_FOLLOW_DIST = Range(15, 25)
param BRAKE_DIST = Range(10, 18)
param EGO_INIT_DIST = Range(40, 60)
param ADV_INIT_DIST = Range(20, 35)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoFollowingBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(0.8)
        do WaitBehavior() for 2 seconds
        resume
    terminate

behavior AdvTurnLeftBehavior(trajectory, merge_speed):
    try:
        do FollowTrajectoryBehavior(target_speed=merge_speed, trajectory=trajectory)
    interrupt when (self.speed >= merge_speed * 0.9):
        take SetThrottleAction(0.6)
        resume
    terminate

behavior OvertakeBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OVERTAKE_SPEED)
    interrupt when (distance from self to ego < 5):
        take SetThrottleAction(0.3)
        resume
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection with multi-lane roads
intersection = Uniform(*filter(lambda i: i.is4Way and len(i.incomingLanes) >= 2, network.intersections))

# Ego goes straight through the intersection
egoInitLane = Uniform(*filter(lambda l: len(l.maneuvers) > 0, intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary comes from the opposite direction and turns left across ego's path
advInitLane = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers)).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Overtaking vehicle starts behind ego in the right adjacent lane
rightLane = Uniform(*filter(lambda l: l is not egoInitLane and l.road is egoInitLane.road, egoInitLane.road.lanes))
overtakeSpawnPt = new OrientedPoint in rightLane.centerline,
    offset by (0, -Range(20, 30)) relative to egoSpawnPt

#################################
# WEATHER AND ENVIRONMENT       #
#################################

param weather = WeatherConditions(
    cloudiness=100,
    precipitation=80,
    precipitationDeposits=90,
    windIntensity=30,
    fogDensity=20,
    wetness=100
)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoFollowingBehavior(egoTrajectory),
    with regionContainedIn None

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color "0,0,0",
    with behavior AdvTurnLeftBehavior(advTrajectory, globalParameters.ADV_SPEED),
    with regionContainedIn None

overtaker = new Car at overtakeSpawnPt,
    with blueprint OVERTAKE_MODEL,
    with color "255,255,255",
    with behavior OvertakeBehavior(),
    with regionContainedIn None

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require (distance from overtakeSpawnPt to egoSpawnPt) >= 15

terminate when (distance from ego to egoSpawnPt) > 120