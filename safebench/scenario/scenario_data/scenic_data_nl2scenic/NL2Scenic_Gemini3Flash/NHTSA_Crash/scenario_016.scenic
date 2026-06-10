"""Scenario Description:

Vehicle A (Ego) is waiting at an intersection to perform a left turn. 
Vehicle B (Adversary) is approaching from the opposite direction, going straight. 
Driver A observes Vehicle B but misjudges the distance and attempts to complete 
the left turn, resulting in a collision where Vehicle B strikes Vehicle A.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.etron"

param EGO_SPEED = Range(4, 6)
param ADV_SPEED = Range(8, 12)

# Parameters to control the timing of the collision
param EGO_WAIT_TIME = Range(1, 2)
param EGO_INIT_DIST = Range(5, 10)
param ADV_INIT_DIST = Range(25, 35)

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    """Ensure both vehicles have a green light to facilitate the intersection conflict."""
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 50):
            setClosestTrafficLightStatus(adversary, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnBehavior(trajectory):
    """Ego waits briefly to simulate observing the adversary, then attempts the turn."""
    # Simulate the "waiting to turn left" state
    take SetBrakeAction(1.0)
    do WaitBehavior() for globalParameters.EGO_WAIT_TIME
    
    # Attempt the turn (misjudging the distance of the oncoming vehicle)
    take SetBrakeAction(0.0)
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

behavior AdvStraightBehavior(trajectory):
    """Adversary drives straight through the intersection at a constant speed."""
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

behavior WaitBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Find a suitable 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# 2. Define Ego's left turn maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 3. Find the Adversary's straight maneuver from the opposite direction
# We look for a straight maneuver that starts on the opposite side of the same road
advInitLane = Uniform(*egoInitLane.group.opposite.lanes)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# 4. Define spawn points based on distance from the intersection
egoSpawnPt = new OrientedPoint following egoInitLane.centerline.orientation from egoInitLane.centerline.end for -globalParameters.EGO_INIT_DIST
advSpawnPt = new OrientedPoint following advInitLane.centerline.orientation from advInitLane.centerline.end for -globalParameters.ADV_INIT_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoLeftTurnBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvStraightBehavior(advTrajectory)

# Ensure the traffic lights don't interfere with the movement
require monitor TrafficLights()

# Verify initial placement logic
require (distance from ego to intersection) < 15
require (distance from adversary to intersection) > 20

# Terminate the scenario if they move far apart or after a collision should have occurred
terminate when (distance from ego to egoSpawnPt) > 50