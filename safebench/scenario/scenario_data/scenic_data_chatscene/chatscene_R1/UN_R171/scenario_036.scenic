'''The ego vehicle approaches an intersection in a straight line while a vehicle target enters from the side at a speed of up to 60 km/h, requiring the ego vehicle to recognize the crossing vehicle and decelerate or stop to provide the right of way and avoid colliding with the target's side'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    lastTurnDirection = None  # Initialize with no previous turn direction.
    
    while (distance to self) > 60:
        wait  # Wait until the vehicle is close enough to influence the ego's path.

    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_TURN_DISTANCE
    # Execute a turn based on the last turn direction.
    if lastTurnDirection == "left":
        take SetSteerAction(globalParameters.OPT_STEER_RIGHT)  # Turn right this time
        lastTurnDirection = "right"
    else:
        take SetSteerAction(globalParameters.OPT_STEER_LEFT)  # Turn left this time
        lastTurnDirection = "left"

    while True:
        take SetSpeedAction(self.speed)  # Continues moving in the new direction

param OPT_ADV_SPEED = Range(5, 15)
param OPT_TURN_DISTANCE = Range(0, 5)
param OPT_STEER_LEFT = Range(-1.0, 0.0)
param OPT_STEER_RIGHT = Range(0.0, 1.0)
intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoManeuver.startLane.centerline

# Setting up the ego vehicle at the initial position
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL

require 10 <= (distance to intersection) <= 40
# Defining adversarial maneuvers as those conflicting with the ego's straight path
advManeuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers)
advManeuver = Uniform(*advManeuvers)
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = advManeuver.connectingLane.centerline[0]  # Initial point on the connecting lane's centerline
IntSpawnPt = advManeuver.connectingLane.centerline.start  # Start of the connecting lane centerline

param OPT_GEO_Y_DISTANCE = Range(-10, 10)
# Setting up the adversarial agent
AdvAgent = Car following roadDirection from IntSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE,
    with heading IntSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

# Requirements to ensure the adversarial agent's relative position and trajectory are correctly aligned with the scenario's needs
require 160 deg <= abs(RelativeHeading(AdvAgent)) <= 180 deg
require any([AdvAgent.position in traj for traj in [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]])