"""Scenario Description:

A blue vehicle approaches a four-way intersection from the bottom lane, intending to execute a left turn as indicated by a blue curved arrow tracing its path into the junction. Ahead of it, a pink vehicle is already positioned within the intersection, angled towards the left with a pink arrow showing its trajectory, acting as a leading object that the blue car is approaching while preparing for its own left turn maneuver.

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

EGO_INIT_DIST = [15, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param LEAD_SPEED = VerifaiRange(3, 6)

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

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead vehicle is already positioned within the intersection on the same left-turn path
leadSpawnPt = new OrientedPoint in egoManeuver.connectingLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color Color.blue,
    with behavior EgoBehavior(egoTrajectory)

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with color Color(1, 0.5, 0.8),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=egoTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST