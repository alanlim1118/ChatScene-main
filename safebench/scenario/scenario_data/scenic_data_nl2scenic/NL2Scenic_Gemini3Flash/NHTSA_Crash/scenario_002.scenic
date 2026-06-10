"""Scenario Description:

Driver ran the red light. The driver saw the light turn yellow but decided to continue through the intersection. 
The ego vehicle is driving legally through the intersection on a green light, while the adversarial vehicle 
approaches from a conflicting direction, ignores the yellow transition, and enters the intersection 
just as the light turns red.

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

EGO_MODEL = "vehicle.audi.tt"
ADV_MODEL = "vehicle.tesla.model3"

param EGO_TARGET_SPEED = Range(7, 10)
param ADV_TARGET_SPEED = Range(10, 13)

# Distances to the intersection center
param EGO_DIST = Range(35, 45)
param ADV_DIST = Range(30, 40)

# Safety threshold for Ego to avoid collision
param SAFETY_DISTANCE = 12

#################################
# MONITORS                      #
#################################

monitor TrafficLightMonitor(ego_vehicle, adv_vehicle, intersection_obj):
    """
    Manages the traffic lights to simulate the 'running a red light' narrative.
    The ego vehicle is given a green light. 
    The adversarial vehicle sees a yellow light as it approaches and a red light as it enters.
    """
    freezeTrafficLights()
    while True:
        # Ego always has green
        setClosestTrafficLightStatus(ego_vehicle, "green")
        
        # Determine distance from adversary to the intersection
        dist_to_int = distance from adv_vehicle to intersection_obj
        
        if dist_to_int > 25:
            # Initially green
            setClosestTrafficLightStatus(adv_vehicle, "green")
        elif 10 < dist_to_int <= 25:
            # Turns yellow - the 'decision' point
            setClosestTrafficLightStatus(adv_vehicle, "yellow")
        else:
            # Turns red as they are about to enter or are inside
            setClosestTrafficLightStatus(adv_vehicle, "red")
        
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    """Ego drives through the intersection but will brake if a collision is imminent."""
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_TARGET_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyCars(self, globalParameters.SAFETY_DISTANCE):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)

behavior AdvBehavior(trajectory):
    """
    Adversarial driver approaches the intersection. 
    Even as the light turns yellow/red (handled by monitor), 
    this agent continues at speed to cross the intersection.
    """
    # Follow trajectory ignores traffic lights by default in Scenic/CARLA unless specified.
    # This represents the driver's decision to "continue through".
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_TARGET_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for a standard 4-way signalized intersection
intersections = filter(lambda i: i.is4Way and i.isSignalized, network.intersections)
intersec = Uniform(*intersections)

# Define Ego's path (Straight)
ego_maneuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, intersec.maneuvers))
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

# Define Adversary's path (Conflicting Straight)
adv_maneuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers))
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

# Placement points on the starting lanes
ego_spawn_pt = ego_maneuver.startLane.centerline[-1]
adv_spawn_pt = adv_maneuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car following roadDirection from ego_spawn_pt for -globalParameters.EGO_DIST,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car following roadDirection from adv_spawn_pt for -globalParameters.ADV_DIST,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(adv_trajectory)

# Activate the traffic light control
require monitor TrafficLightMonitor(ego, adversary, intersec)

# Ensure the scenario terminates eventually
terminate when (distance from ego to ego_maneuver.endLane) < 5 and (distance from adversary to adv_maneuver.endLane) < 5