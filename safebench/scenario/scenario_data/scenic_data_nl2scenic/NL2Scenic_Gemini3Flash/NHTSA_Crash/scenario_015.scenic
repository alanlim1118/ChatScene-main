"""Scenario Description:

A northbound vehicle, A, was waiting to make a left turn. 
The light changed and the northbound vehicle began to turn left. 
A southbound driver, B, accelerated hard, hoping to make the light and struck Vehicle A.

This is modeled at a 4-way signalized intersection.
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

# Models
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.etron"

# Speeds (m/s)
param EGO_TURN_SPEED = Range(4, 6)
param ADV_TARGET_SPEED = Range(15, 20)  # "Accelerated hard"

# Distances
param EGO_DIST_TO_INTERSECTION = Range(2, 5)
param ADV_DIST_TO_INTERSECTION = Range(25, 35)

#################################
# MONITORS                      #
#################################

monitor TrafficLightCycle(intersection):
    """
    Controls traffic lights to simulate the 'light changed' trigger.
    Starts red for North/South, then switches to green.
    """
    freezeTrafficLights()
    # Ensure the intersection lights for our vehicles start red
    setClosestTrafficLightStatus(ego, "red")
    setClosestTrafficLightStatus(adv, "red")
    
    wait for 3 seconds  # Vehicles wait at the red light
    
    # Change to green
    setClosestTrafficLightStatus(ego, "green")
    setClosestTrafficLightStatus(adv, "green")
    
    while True:
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnBehavior(trajectory, target_speed):
    """
    Wait for the light to turn green, then execute a left turn.
    """
    # Wait until the light turns green
    while getClosestTrafficLightStatus(self) != "green":
        take SetBrakeAction(1.0)
    
    # Proceed with the turn
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
    terminate

behavior AdvHardAccelerateBehavior(target_speed):
    """
    Wait for the light to turn green, then accelerate hard straight through the intersection.
    """
    # Wait until the light turns green
    while getClosestTrafficLightStatus(self) != "green":
        take SetBrakeAction(1.0)
    
    # "Accelerate hard" by driving straight at a high target speed
    # We use FollowLaneBehavior as the adversary is going straight
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Select a 4-way signalized intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# 2. Select a left-turn maneuver for the Ego (Vehicle A)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 3. Select a straight maneuver for the Adversary (Vehicle B) 
# It must come from the opposite direction (Southbound to Ego's Northbound)
advManeuver = Uniform(*filter(lambda m: (m.type is ManeuverType.STRAIGHT and 
                                       m.startLane.group.opposite == egoInitLane.group), 
                             intersection.maneuvers))
advInitLane = advManeuver.startLane

# Define spawn points back from the intersection stop lines
egoSpawnPt = OrientedPoint at egoInitLane.centerline.end offset along egoInitLane.centerline.end.heading by -globalParameters.EGO_DIST_TO_INTERSECTION
advSpawnPt = OrientedPoint at advInitLane.centerline.end offset along advInitLane.centerline.end.heading by -globalParameters.ADV_DIST_TO_INTERSECTION

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoLeftTurnBehavior(trajectory=egoTrajectory, target_speed=globalParameters.EGO_TURN_SPEED)

adv = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvHardAccelerateBehavior(target_speed=globalParameters.ADV_TARGET_SPEED)

# Run the traffic light monitor
require monitor TrafficLightCycle(intersection)

# Ensure the simulation terminates if they get too far or after a reasonable time
terminate when (distance from ego to intersection) > 50