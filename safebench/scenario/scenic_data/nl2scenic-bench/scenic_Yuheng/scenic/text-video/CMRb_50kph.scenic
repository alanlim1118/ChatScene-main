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
MOTO_MODEL = "vehicle.yamaha.yzf"

param OPT_MOTO_SPEED = Range(3, 6)
param OPT_VEH_SPEED = globalParameters.OPT_MOTO_SPEED * Uniform(1.4, 1.6, 1.8)
param OPT_INITIAL_GAP = Range(25, 40)
param OPT_BRAKE_DIST = Range(15, 25)
param OPT_LANE_OFFSET = Range(0.8, 1.2)

#################################
# AGENT BEHAVIORS               #
#################################

behavior MotorcycleBehavior(speed, brakeDist):
    """Motorcycle travels at constant speed then decelerates."""
    do FollowLaneBehavior(speed) until (distance from self to ego) < brakeDist
    take SetThrottleAction(0)
    take SetBrakeAction(0.8)
    do FollowLaneBehavior(0.5) for 3 seconds
    take SetBrakeAction(1)
    terminate

behavior RearEndBehavior(speed, trajectory):
    """Vehicle approaches from behind at higher speed without braking."""
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment for the rear-end scenario
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 2, network.roadSegments))
targetLane = Uniform(*roadSegment.lanes)
leftLane = targetLane._laneToLeft

require leftLane is not None

# Define spawn points along the lane centerlines
motoSpawnPt = new OrientedPoint in targetLane.centerline
vehOffsetPt = new OrientedPoint following roadDirection from motoSpawnPt for -globalParameters.OPT_INITIAL_GAP

# Project vehicle spawn point into the left lane
vehProjectPt = leftLane.centerline.project(vehOffsetPt.position)
vehHeading = leftLane.orientation[vehProjectPt]

# Build trajectories
motoTrajectory = [targetLane]
vehTrajectory = [leftLane]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Motorcycle at motoSpawnPt,
    with blueprint MOTO_MODEL,
    with regionContainedIn None,
    with behavior MotorcycleBehavior(globalParameters.OPT_MOTO_SPEED, globalParameters.OPT_BRAKE_DIST)

AdvAgent = new Car at vehProjectPt,
    with heading vehHeading,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior RearEndBehavior(globalParameters.OPT_VEH_SPEED, vehTrajectory)

# Ensure the vehicle starts behind the motorcycle in the left lane
require (distance from AdvAgent to ego) >= globalParameters.OPT_INITIAL_GAP - 5
require (distance from AdvAgent to ego) <= globalParameters.OPT_INITIAL_GAP + 5