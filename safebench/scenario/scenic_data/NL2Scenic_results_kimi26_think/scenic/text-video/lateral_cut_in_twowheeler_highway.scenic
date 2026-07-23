"""Scenario Description:

The ego vehicle is driving forward under wet weather conditions on an elevated highway or bridge, approaching a motorcycle carrying two riders ahead in the same lane. As the ego vehicle gets closer, the motorcyclist loses control on the slick road surface and falls over, dropping the bike and both riders directly into the ego vehicle's path. The ego vehicle is forced into an emergency braking scenario but ultimately results in a rear-end collision with the fallen two-wheeled vehicle and its occupants lying on the road.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
param weather = "WetCloudyNoon"
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_MOTO_SPEED = Range(6, 9)
param OPT_SPAWN_DIST = Range(40, 60)
param OPT_FALL_DIST = Range(10, 15)
param OPT_BRAKE_DIST = Range(10, 18)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EmergencyBrakeBehavior():
    while True:
        take SetBrakeAction(1.0)
        take SetThrottleAction(0.0)

behavior FallenMotorcycleBehavior(fall_reference):
    # Drive forward until near the fall location, then lose control
    do FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED) until (distance from self to fall_reference) < 2
    # Simulate losing control on slick surface: sharp steer and hard brake
    take SetSteerAction(1.0)
    take SetBrakeAction(1.0)
    take SetThrottleAction(0.0)
    # Remain fallen in the lane as an obstacle
    do WaitBehavior()

behavior EgoEmergencyBehavior(adversary):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to adversary) < globalParameters.OPT_BRAKE_DIST:
        do EmergencyBrakeBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Select a long, straight forward lane to simulate an elevated highway/bridge
candidateLanes = filter(lambda l: l.isForward and l.length > 150, network.lanes)
egoLane = Uniform(*candidateLanes)

egoSpawnPt = new OrientedPoint in egoLane.centerline

# Spawn motorcycle ahead in the same lane
motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_SPAWN_DIST

# Point where the motorcycle loses control and falls, dropping riders into the path
fallPt = new OrientedPoint following roadDirection from motoSpawnPt for globalParameters.OPT_FALL_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Motorcycle ahead in the same lane
motorcycle = new Motorcycle at motoSpawnPt,
    with heading motoSpawnPt.heading,
    with regionContainedIn None,
    with behavior FallenMotorcycleBehavior(fallPt)

# Two riders represented as persons dropped in the path at the fall location
rider1 = new Person at fallPt offset by (0, -0.5),
    with regionContainedIn None,
    with behavior WaitBehavior()

rider2 = new Person at fallPt offset by (0, 0.5),
    with regionContainedIn None,
    with behavior WaitBehavior()

# Ego vehicle approaching from behind
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoEmergencyBehavior(motorcycle)