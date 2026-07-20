'''The ego vehicle travels along a straight path while a bicycle target, initially obstructed during its acceleration phase, emerges and crosses perpendicularly at a constant speed of 15 km/h, the ego vehicle should detect the bicycle target and execute an autonomous braking maneuver to avoid a collision'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    do CrossingBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE)
    
    while True:
        # Set speed to a random value within the allowable range at each cycle
        speedChange = globalParameters.OPT_SPEED_CHANGE  # Random speed adjustment
        take SetSpeedAction(speedChange)

        # Wait for a number of steps as defined by a Range
        for _ in range(globalParameters.OPT_WAIT_STEPS):
            wait

param OPT_ADV_SPEED = Range(0, 10)  # Maximum allowable speed change
param OPT_ADV_DISTANCE = Range(10, 20)  # Distance threshold
param OPT_SPEED_CHANGE = Range(0, 10)  # Allows random speed setting up to the maximum defined speed
param OPT_WAIT_STEPS = Range(5, 30)  # Wait time in steps
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Setting up the ego vehicle at the initial position
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL

require 10 <= (distance to intersection) <= 40
# Parameters for scenario elements
param OPT_GEO_BLOCKER_Y_DISTANCE = Range(0, 40)
param OPT_GEO_X_DISTANCE = Range(-8, 0)  # Offset for the agent in the opposite lane
param OPT_GEO_Y_DISTANCE = Range(10, 30)

# Setting up the parked car that blocks the ego's path
laneSec = network.laneSectionAt(ego)  # Assuming network.laneSectionAt(ego) is predefined in the geometry part
IntSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
Blocker = Car at IntSpawnPt,
    with heading IntSpawnPt.heading,
    with regionContainedIn None

# Setup for the motorcyclist who unexpectedly enters the scene
SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Bicycle at Blocker offset along IntSpawnPt.heading by SHIFT,
    with heading IntSpawnPt.heading + 180 deg,  # The agent is facing the opposite direction, indicating oncoming
    with regionContainedIn laneSec._laneToLeft,  # Positioned in the left lane, assuming it's the oncoming traffic lane
    with behavior AdvBehavior()