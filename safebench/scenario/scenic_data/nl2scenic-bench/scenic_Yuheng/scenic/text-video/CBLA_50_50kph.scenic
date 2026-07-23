"""Scenario Description:

The ego vehicle travels forward at a constant speed of 50 km/h along a straight road. A bicyclist is positioned ahead in the same lane, traveling in the same direction but at a slower speed. The ego vehicle maintains its speed without braking or steering, resulting in a collision course where it rapidly closes the distance to the cyclist.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 50 km/h ≈ 13.89 m/s
EGO_SPEED = 13.89
# Cyclist slower than ego to ensure closing distance
CYCLIST_SPEED = Range(3.0, 6.0)
# Initial distance between ego and cyclist
INITIAL_GAP = Range(30, 50)

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantSpeedBehavior(speed):
    """Maintain constant speed without braking or evasive steering."""
    while True:
        take SetThrottleAction(1.0)
        take SetBrakeAction(0.0)
        take SetSteerAction(0.0)

behavior CyclistFollowLaneBehavior(speed):
    """Cyclist follows the lane at a constant slower speed."""
    do FollowLaneBehavior(speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment suitable for this scenario
lane = Uniform(*filter(lambda l: l.isForward and l.length > 100, network.lanes))

egoSpawnPt = new OrientedPoint in lane.centerline

# Place cyclist ahead of ego in the same lane
cyclistSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for INITIAL_GAP,
    with heading lane.orientation[lane.centerline.project(egoSpawnPt.position + (INITIAL_GAP * Vector(1,0).rotatedBy(lane.orientation[lane.centerline.project(egoSpawnPt.position)]))]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior ConstantSpeedBehavior(EGO_SPEED),
    with speed EGO_SPEED

cyclist = new Bicycle at cyclistSpawnPt,
    with heading lane.orientation[lane.centerline.project(cyclistSpawnPt.position)],
    with regionContainedIn None,
    with behavior CyclistFollowLaneBehavior(CYCLIST_SPEED),
    with speed CYCLIST_SPEED

# Ensure both agents are on the same lane and properly spaced
require distance from ego to cyclist >= 25
require distance from ego to cyclist <= 60

# Terminate after sufficient time for collision to occur
terminate after 10 seconds