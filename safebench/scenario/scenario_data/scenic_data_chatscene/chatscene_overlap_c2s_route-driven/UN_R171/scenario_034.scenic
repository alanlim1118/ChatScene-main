'''While the ego vehicle travels at its prescribed test speed through an intersection, a bicycle target emerges from an obstructed area and crosses perpendicular to the ego vehicle's path at a constant speed of 15 km/h, timed precisely so that its centerline aligns with the ego vehicle's impact point, requiring the ego vehicle to detect the cyclist and execute an autonomous maneuver to avoid a collision.'''
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

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
# Parameters for scenario elements
param OPT_GEO_BLOCKER_Y_DISTANCE = Range(0, 40)
param OPT_GEO_X_DISTANCE = Range(-2, 2)
param OPT_GEO_Y_DISTANCE = Range(2, 6)

# Setup for the blocking car that the ego must bypass
laneSec = network.laneSectionAt(ego)
IntSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
Blocker = Car at IntSpawnPt,
    with heading IntSpawnPt.heading,
    with regionContainedIn None

# Setup for the pedestrian who suddenly appears and complicates the maneuver
SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Bicycle at Blocker offset along IntSpawnPt.heading by SHIFT,
    with heading IntSpawnPt.heading + 90 deg,  # Perpendicular to the road, crossing the street
    with regionContainedIn None,
    with behavior AdvBehavior()