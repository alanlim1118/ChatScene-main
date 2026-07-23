"""Scenario Description:

The ego vehicle travels forward at a constant speed of 50.00 km/h along a grey road surface bordered by a green strip at the top. A bicyclist is traveling in the same direction ahead of the ego vehicle. The ego vehicle rapidly closes the distance to the cyclist without applying braking or initiating evasive steering, resulting in a rear-end collision with the bicyclist in front.

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
EGO_SPEED = 13.89  # 50 km/h in m/s
CYCLIST_SPEED = Range(2.0, 4.0)  # m/s

param OPT_SPAWN_DISTANCE = Range(25, 45)

#################################
# AGENT BEHAVIORS               #
#################################

behavior DriveForwardBehavior(speed):
    do FollowLaneBehavior(speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane for the straight road scenario
lane = Uniform(*network.lanes)

# Spawn the bicyclist ahead on the lane centerline
cyclistSpawnPt = new OrientedPoint in lane.centerline

# Spawn the ego vehicle behind the bicyclist
egoSpawnPt = new OrientedPoint following roadDirection from cyclistSpawnPt for -globalParameters.OPT_SPAWN_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading lane.orientation[egoSpawnPt],
    with blueprint EGO_MODEL,
    with behavior DriveForwardBehavior(EGO_SPEED),
    with regionContainedIn None

bicyclist = new Bicycle at cyclistSpawnPt offset by 0 @ 0.8,
    with heading lane.orientation[cyclistSpawnPt],
    with behavior DriveForwardBehavior(CYCLIST_SPEED),
    with regionContainedIn None

# Ensure the lane is long enough for the scenario
require lane.length > 100
require (distance from ego to bicyclist) > 20

# Terminate when the ego collides with or comes very close to the bicyclist
terminate when (distance from ego to bicyclist) < 2.0
terminate after 20 seconds