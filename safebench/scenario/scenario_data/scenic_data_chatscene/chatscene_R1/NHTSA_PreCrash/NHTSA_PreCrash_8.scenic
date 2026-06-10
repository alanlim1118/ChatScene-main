'''Vehicle is turning left in an urban area, in daylight, under clear weather conditions with a posted speed limit of 35 mph; and encounters a pedestrian in the crosswalk at a signaled intersection'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # The adversarial vehicle waits until it is within a threatening proximity.
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_STEER_DISTANCE
    # Once close to the ego's trajectory, the adversarial vehicle initiates a sharp turn.
    while True:
        take SetSpeedAction(self.speed)  # Continues moving at its current speed
        take SetSteerAction(globalParameters.OPT_STEER)  # Makes an abrupt steering adjustment

param OPT_ADV_SPEED = Range(5, 15)  # The speed at which the adversarial vehicle approaches the intersection.
param OPT_STEER_DISTANCE = Range(0, 4)  # Distance from ego's trajectory at which the vehicle starts to turn.
param OPT_STEER = Range(-1.0, -0.5)  # Defines the intensity of the steering action.
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
# Parameters for scenario elements
param OPT_GEO_BLOCKER_Y_DISTANCE = Range(0, 40)
param OPT_GEO_X_DISTANCE = Range(-2, 2)
param OPT_GEO_Y_DISTANCE = Range(2, 6)

# Setup for the blocking car that the ego must bypass
laneSec = network.laneSectionAt(ego)
IntSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
Blocker = Car at IntSpawnPt,
    with heading IntSpawnPt.heading,
    with regionContainedIn None

# Setup for the pedestrian who suddenly appears and complicates the maneuver
SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Pedestrian at Blocker offset along IntSpawnPt.heading by SHIFT,
    with heading IntSpawnPt.heading + 90 deg,  # Perpendicular to the road, crossing the street
    with regionContainedIn None,
    with behavior AdvBehavior()