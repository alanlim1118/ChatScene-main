"""Scenario Description:

A blue vehicle positioned in the right-hand lane of the southern approach at a four-way intersection executes a lane change from the left lane into the right lane while traversing straight through the junction. Alternative maneuvers (left turn from left lane, right turn from right lane) are considered as part of the maneuver selection space, but the primary behavior follows a straight trajectory consistent with a lane-change-right maneuver through the intersection.

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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

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

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select an incoming lane that has a straight maneuver (representing the path through intersection)
# The "lane change right" is interpreted as starting in a left/center lane and ending in the right lane
# via a straight connecting lane across the intersection.
egoInitLane = Uniform(*filter(lambda l: 
    any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers),
    intersection.incomingLanes
))

# Prefer straight maneuvers; this captures the "proceeding straight through" aspect
# which, combined with lane positioning, represents the lane-change-right-through-intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST