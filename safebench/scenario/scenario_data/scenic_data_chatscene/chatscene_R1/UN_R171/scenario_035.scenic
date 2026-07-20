'''The ego vehicle approaches an intersection in a straight line and initiates a turn across the path of an oncoming passenger car or motorcycle traveling at up to 60 km/h, requiring the system to detect the oncoming threat and intervene to prevent a collision'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    speedCycle = [globalParameters.OPT_HIGH_SPEED, globalParameters.OPT_LOW_SPEED] * 10  # Predefined speed cycle
    
    while (distance to self) > 60:
        wait  # Wait until within 60 meters

    for speed in speedCycle:
        take SetSpeedAction(speed)

        # Wait for a dynamically determined duration before changing speed again
        for _ in range(globalParameters.OPT_WAIT_SPEED_CHANGE):
            wait

param OPT_HIGH_SPEED = Range(10, 20)  # Upper bound speed
param OPT_LOW_SPEED = Range(1, 5)  # Lower bound speed
param OPT_WAIT_SPEED_CHANGE = Range(5, 40)  # Variable wait time before changing speed
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
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
AdvAgent = Car or Motorcycle at Blocker offset along IntSpawnPt.heading by SHIFT,
    with heading IntSpawnPt.heading + 180 deg,  # The agent is facing the opposite direction, indicating oncoming
    with regionContainedIn laneSec._laneToLeft,  # Positioned in the left lane, assuming it's the oncoming traffic lane
    with behavior AdvBehavior()