"""Scenario Description:

Under clear, sunny conditions in an urban environment, the ego vehicle proceeds along a multi-lane road separated by a tree-lined median, approaching a large roundabout with a central architectural structure. A red adversary vehicle travels in the adjacent left lane, moving concurrently alongside the ego vehicle as both approach the intersection. Upon reaching the entrance, both vehicles enter the roundabout simultaneously, with the red car occupying the inner circulating lane while the ego vehicle stays in the outer lane to execute a leftward path. The ego vehicle must maintain strict lane discipline and continuously monitor the adjacent red vehicle to safely navigate the curve and complete its turn through the roundabout without conflict.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(6, 9)
param ADV_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 120

EGO_INIT_DIST = [30, 45]
ADV_LATERAL_OFFSET = Range(3.0, 4.0)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a roundabout intersection from the network
roundabout = Uniform(*filter(lambda i: hasattr(i, 'isRoundabout') and i.isRoundabout, network.intersections))

# Ego takes the outer lane maneuver (left turn / circumnavigation)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, roundabout.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary spawns in the adjacent left lane at similar longitudinal position
advInitLane = Uniform(*filter(lambda l: l is not egoInitLane and l.road is egoInitLane.road, egoInitLane.leftLanes))
advManeuverCandidates = filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers)
advManeuver = Uniform(*advManeuverCandidates) if advManeuverCandidates else egoManeuver
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Clear sunny weather
param weather = WeatherConditions(
    cloudiness=0,
    precipitation=0,
    precipitation_deposits=0,
    wind_intensity=0,
    sun_azimuth_angle=45,
    sun_altitude_angle=70,
    fog_density=0,
    wetness=0
)

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with regionContainedIn None

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (0.8, 0.0, 0.0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory),
    with regionContainedIn None

# Ensure ego starts at appropriate distance from roundabout
require EGO_INIT_DIST[0] <= (distance from ego to roundabout) <= EGO_INIT_DIST[1]

# Ensure adversary is roughly alongside ego longitudinally
require abs((distance from adversary to roundabout) - (distance from ego to roundabout)) <= 5

terminate when (distance from ego to egoSpawnPt) > TERM_DIST