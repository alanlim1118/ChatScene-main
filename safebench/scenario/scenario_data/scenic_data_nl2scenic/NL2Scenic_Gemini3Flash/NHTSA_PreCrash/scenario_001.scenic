"""Scenario Description:

Vehicle is turning left or right at an intersection-related area, in daylight, 
under clear weather conditions, with a posted speed limit of 45 mph or less, 
and then loses control due to wet or slippery roads and runs off the road.

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

# Selecting 'WetNoon' to represent daylight, clear sky (no active rain), but wet/slippery roads.
param weather = 'WetNoon'

EGO_MODEL = "vehicle.audi.tt"
param OPT_EGO_SPEED = Range(7, 10) # Speed during the turn (m/s)
param OPT_LOSE_CONTROL_STEER = Uniform(Range(-1.0, -0.7), Range(0.7, 1.0)) # Aggressive steering to simulate spin
param OPT_LOSE_CONTROL_THROTTLE = Range(0.8, 1.0)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior LossOfControlBehavior(trajectory):
    # Phase 1: Follow the lane/trajectory towards the intersection
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.OPT_EGO_SPEED) \
            until (self.intersection is not None)
    except:
        pass

    # Phase 2: Once in the intersection/turn, simulate loss of control
    # We wait a short duration into the turn to initiate the 'slip'
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.OPT_EGO_SPEED) for Range(0.5, 1.5) seconds
    
    # Phase 3: The actual loss of control - sudden steering and high throttle on wet pavement
    take SetSteerAction(globalParameters.OPT_LOSE_CONTROL_STEER)
    take SetThrottleAction(globalParameters.OPT_LOSE_CONTROL_THROTTLE)
    
    # Observe the car spinning off the road for a few seconds
    do WaitBehavior() for 4 seconds
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find all 3-way or 4-way intersections
intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersec = Uniform(*intersections)

# Filter maneuvers for Left or Right turns
turning_maneuvers = filter(lambda m: m.type in [ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN], intersec.maneuvers)
egoManeuver = Uniform(*turning_maneuvers)

# Define trajectory: [Start Lane, Connecting Lane (in intersection), End Lane]
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Define starting point on the lane leading to the intersection
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint following egoInitLane.orientation from egoInitLane.centerline.end for Range(-25, -15)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior LossOfControlBehavior(egoTrajectory)

# Ensure the road selected is part of a standard urban area (which usually implies < 45mph)
require egoManeuver.startLane.speedLimit <= 20.12 # 20.12 m/s is approx 45 mph

# Termination condition: stop when the car has finished its behavior
terminate when (distance from ego to egoSpawnPt) > 50