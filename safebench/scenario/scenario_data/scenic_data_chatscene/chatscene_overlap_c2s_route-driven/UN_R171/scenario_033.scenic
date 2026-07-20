'''The ego vehicle travels at a constant test speed through an intersection while a pedestrian target begins crossing from the near side at 5 km/h on a trajectory timed to impact the VUT's longitudinal centerline, requiring the system to detect the target at least 4 seconds before the predicted collision and intervene to avoid the impact'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    direction = self.heading + globalParameters.OPT_ADV_DEGREE deg
    while (distance to self) > globalParameters.OPT_ADV_DISTANCE:
        wait
    while True:
        take SetWalkingDirectionAction(direction)
        take SetWalkingSpeedAction(globalParameters.OPT_ADV_SPEED)

param OPT_ADV_SPEED = Range(0, 5)
param OPT_ADV_DISTANCE = Range(0, 15)
param OPT_ADV_DEGREE = Range(-90, 90)
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
# Defining adversarial maneuvers as those conflicting with the ego's straight path
advManeuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers)
advManeuver = Uniform(*advManeuvers)
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = advManeuver.connectingLane.centerline[0]  # Initial point on the connecting lane's centerline
IntSpawnPt = advManeuver.connectingLane.centerline.start  # Start of the connecting lane centerline

param OPT_GEO_Y_DISTANCE = Range(-10, 10)
# Setting up the adversarial agent
AdvAgent = Pedestrian following roadDirection from IntSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE,
    with heading IntSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

# Requirements to ensure the adversarial agent's relative position and trajectory are correctly aligned with the scenario's needs
require 160 deg <= abs(RelativeHeading(AdvAgent)) <= 180 deg
require any([AdvAgent.position in traj for traj in [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]])