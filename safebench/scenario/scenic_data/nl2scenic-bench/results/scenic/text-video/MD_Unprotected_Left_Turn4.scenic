"""Scenario Description:

The ego vehicle executes a left turn at a foggy, urban four-way intersection flanked by tall buildings, viewed from a high-angle perspective that rotates during the sequence. As the ego vehicle moves through the center of the junction, it navigates the turn while two oncoming vehicles from the opposite direction proceed straight through the intersection. Simultaneously, a third vehicle approaches from the left arm and drives straight across the junction, passing perpendicular to the ego vehicle's path. The road is marked with stop lines and crosswalks, and the heavy fog reduces visibility across the multi-lane roads and surrounding cityscape.

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

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(6, 9)

ONCOMING_INIT_DIST = [25, 40]
param ONCOMING_SPEED = VerifaiRange(7, 10)

LATERAL_INIT_DIST = [20, 35]
param LATERAL_SPEED = VerifaiRange(7, 10)

TERM_DIST = 80

# Fog density for heavy fog conditions
param FOG_DENSITY = VerifaiRange(0.03, 0.06)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego vehicle: left turn maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Oncoming vehicles (2): from opposite direction, going straight
oncomingLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoInitLane.reverseManeuvers)
    ).startLane
oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, oncomingLane.maneuvers))
oncomingTrajectory = [oncomingLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt1 = new OrientedPoint in oncomingLane.centerline
oncomingSpawnPt2 = new OrientedPoint in oncomingLane.centerline

# Lateral vehicle: from the left arm relative to ego, going straight
# The left arm is identified via conflicting maneuvers that are perpendicular
leftArmLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        filter(lambda cm: 
            abs(relativeHeading(cm.startLane, egoInitLane)) > 0.7,
            egoManeuver.conflictingManeuvers)
    )).startLane
lateralManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leftArmLane.maneuvers))
lateralTrajectory = [leftArmLane, lateralManeuver.connectingLane, lateralManeuver.endLane]
lateralSpawnPt = new OrientedPoint in leftArmLane.centerline

#################################
# CAMERA SETUP                  #
#################################

# High-angle rotating camera centered above intersection
cameraCenter = new OrientedPoint at intersection.center,
    with pitch -60 deg @ (-45 deg, 0 deg)

behavior RotatingCameraBehavior():
    do SetCameraPositionAction(position=cameraCenter.positionOffset(0, 0, 40))
    do SetCameraRotationAction(rotation=Vector(0, 0, globalParameters._time * 10))

camera = new Object at cameraCenter,
    with behavior RotatingCameraBehavior()

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set weather to heavy fog
weather = new Weather with fogDensity globalParameters.FOG_DENSITY

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoLeftTurnBehavior(egoTrajectory)

# Two oncoming vehicles going straight
oncoming1 = new Car at oncomingSpawnPt1,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=oncomingTrajectory)

oncoming2 = new Car at oncomingSpawnPt2,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=oncomingTrajectory)

# Lateral vehicle from left arm going straight
lateral = new Car at lateralSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LATERAL_SPEED, trajectory=lateralTrajectory)

# Spatial constraints
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ONCOMING_INIT_DIST[0] <= (distance from oncoming1 to intersection) <= ONCOMING_INIT_DIST[1]
require ONCOMING_INIT_DIST[0] <= (distance from oncoming2 to intersection) <= ONCOMING_INIT_DIST[1]
require LATERAL_INIT_DIST[0] <= (distance from lateral to intersection) <= LATERAL_INIT_DIST[1]

# Ensure oncoming vehicles are spaced apart
require (distance from oncoming1 to oncoming2) >= 10

# Termination condition
terminate when (distance from ego to egoSpawnPt) > TERM_DIST