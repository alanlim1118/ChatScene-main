"""Scenario Description:

At a T-junction, the ego vehicle, represented by a green box, is positioned on the vertical road and initiates a right-hand turn onto the horizontal intersecting road. Simultaneously, an adversary vehicle, depicted as a yellow box, approaches from the opposing arm of the vertical road and executes a left-hand turn into the same intersecting road. As the camera perspective shifts and rotates, both vehicles are seen converging towards the intersection, with the green vehicle completing its turn to the right and the yellow vehicle turning left to enter the same roadway, effectively merging into the same lane from opposite directions.

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
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [20, 30]
param ADV_SPEED = VerifaiRange(6, 9)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 80

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

# Select a 3-way (T-junction) intersection
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego makes a right turn at the T-junction
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary comes from the opposing arm of the vertical road and turns left
# into the same road that ego is turning right into.
# The opposing arm corresponds to reverse maneuvers of STRAIGHT from ego's start lane,
# but at a T-junction we look for LEFT_TURN maneuvers whose endLane matches ego's endLane.
advManeuver = Uniform(*filter(lambda m: 
        m.type is ManeuverType.LEFT_TURN and m.endLane is egoManeuver.endLane,
        intersection.maneuvers
    ))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color "0,255,0",
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color "255,255,0",
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]

# Ensure both vehicles merge into the same lane
require advManeuver.endLane is egoManeuver.endLane

terminate when (distance from ego to egoSpawnPt) > TERM_DIST