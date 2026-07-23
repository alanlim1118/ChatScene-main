"""Scenario Description:

In a top-down simulation view of a roadway with a grey surface and a green verge, a traffic collision scenario unfolds involving two agents. A motorcyclist, represented by a white rectangular bounding box with black stripes, is traveling forward in the lane. Approaching from behind on the left is a vehicle, depicted as a small black dash, moving at a higher speed. As the sequence progresses, the vehicle rapidly closes the distance to the motorcyclist. According to the scenario description, the motorcyclist travels at a constant speed before decelerating, while the vehicle continues forward. The sequence culminates in a rear-end collision where the frontal structure of the faster-moving vehicle strikes the rear of the motorcyclist, with the vehicle eventually overlapping the space occupied by the motorcycle.

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

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.5, 2.0, 2.5)
param OPT_SPAWN_DISTANCE = Range(15, 30)
param OPT_DECEL_DELAY = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantSpeedThenDecelerateBehavior(speed, decel_delay):
    """Travel at a constant speed for a given duration, then brake abruptly."""
    do FollowLaneBehavior(target_speed=speed) for decel_delay seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    while True:
        wait

behavior FollowAtHighSpeedBehavior(speed):
    """Continue forward at a higher speed without braking."""
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane for the scenario
lane = Uniform(*network.lanes)

# Spawn the motorcyclist along the lane centerline
motorcycleSpawnPt = new OrientedPoint in lane.centerline

# Spawn the approaching vehicle behind the motorcyclist
carSpawnPt = new OrientedPoint ahead of motorcycleSpawnPt by -globalParameters.OPT_SPAWN_DISTANCE

# Ensure sufficient road length remains for the scenario to unfold
require (distance from motorcycleSpawnPt to lane.centerline.end) > globalParameters.OPT_SPAWN_DISTANCE + 20

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Motorcycle at motorcycleSpawnPt,
    with heading motorcycleSpawnPt.heading,
    with regionContainedIn None,
    with behavior ConstantSpeedThenDecelerateBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_DECEL_DELAY)

AdvAgent = new Car at carSpawnPt,
    with heading carSpawnPt.heading,
    with regionContainedIn None,
    with behavior FollowAtHighSpeedBehavior(globalParameters.OPT_ADV_SPEED)

# Ensure the adversarial vehicle is positioned behind the motorcyclist
require (distance from AdvAgent to ego) > 0