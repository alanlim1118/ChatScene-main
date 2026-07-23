"""Scenario Description:

In a top-down schematic view of a four-way intersection, a blue ego vehicle is positioned in the vertical lane facing north, situated just below the center of the junction. Approaching from the left horizontal road is a pink adversarial vehicle, which is depicted facing east. The pink vehicle executes a right turn, steering from the left road into the downward vertical lane, passing on the left moving towards ego while turning right.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [15, 25]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(6, 9)

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

# Ego is in a vertical lane facing north, just below the intersection center
# This means ego is on a south-to-north incoming lane
egoInitLane = Uniform(*filter(lambda l: l.heading >= -45 deg and l.heading <= 45 deg, intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary approaches from the left horizontal road (west-to-east), so it's on a lane heading ~0 deg (east)
# relative to ego's northbound lane, the adversary's lane is to the left
# The adversary makes a RIGHT TURN into the downward (southbound) vertical lane
advInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.RIGHT_TURN,
        egoInitLane.leftAdjacentLane.maneuvers if egoInitLane.leftAdjacentLane else []
    )).startLane if egoInitLane.leftAdjacentLane else \
    Uniform(*filter(lambda l: 
        abs(l.heading - egoInitLane.heading + 90 deg) < 30 deg or abs(l.heading - egoInitLane.heading - 270 deg) < 30 deg,
        intersection.incomingLanes))

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "blue",
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color "pink",
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]
terminate when (distance from ego to egoSpawnPt) > TERM_DIST