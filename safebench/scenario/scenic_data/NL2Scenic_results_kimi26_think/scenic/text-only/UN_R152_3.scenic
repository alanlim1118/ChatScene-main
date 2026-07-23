"""Scenario Description:

The subject vehicle drives a small radius curved road of which the guard pipes are constructed to the outer side, and a stationary vehicle (M1 category), a stationary pedestrian target or a stationary bicycle target is positioned just outside of the guard pipes and where on the extension of the centre of the lane.

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

param OPT_EGO_SPEED = Range(5, 15)
param OPT_GUARD_LATERAL_OFFSET = Range(3, 5)
param OPT_TARGET_LATERAL_OFFSET = Range(1, 2)
param OPT_BRAKE_DIST = Range(10, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*network.intersections)
turnManeuver = Uniform(*filter(lambda m: m.type in (ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN), intersection.maneuvers))
egoInitLane = turnManeuver.startLane
egoTrajectory = [egoInitLane, turnManeuver.connectingLane, turnManeuver.endLane]
lane = turnManeuver.connectingLane

# Point along the curved connecting lane where the guard pipes and target are located
scenarioPt = new OrientedPoint in lane.centerline

# Place guard pipes on the outer side of the curve and target just outside them
if turnManeuver.type is ManeuverType.LEFT_TURN:
    # Left turn: outer side is to the right
    guardPipePt = new OrientedPoint right of scenarioPt by globalParameters.OPT_GUARD_LATERAL_OFFSET
    targetPt = new OrientedPoint right of guardPipePt by globalParameters.OPT_TARGET_LATERAL_OFFSET
else:
    # Right turn: outer side is to the left
    guardPipePt = new OrientedPoint left of scenarioPt by globalParameters.OPT_GUARD_LATERAL_OFFSET
    targetPt = new OrientedPoint left of guardPipePt by globalParameters.OPT_TARGET_LATERAL_OFFSET

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Guard pipes (barrier) constructed to the outer side of the curved road
GuardPipes = new Barrier at guardPipePt,
    with heading scenarioPt.heading,
    with regionContainedIn None

# Stationary target just outside of the guard pipes on the extension of the lane centre
TargetClass = Uniform(Car, Pedestrian, Bicycle)
AdvAgent = new TargetClass at targetPt,
    with heading scenarioPt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior()

require 40 <= (distance to intersection) <= 60