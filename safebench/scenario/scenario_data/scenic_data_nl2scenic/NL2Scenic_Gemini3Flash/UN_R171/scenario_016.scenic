"""Scenario Description:

The ego vehicle navigates a straight section of the road and encounters a much slower vehicle target ahead 
where the speed differential between the two vehicles is significantly increased beyond 50 km/h 
(set to ~90 km/h) to test the system's high-speed closing response.

"""

#################################
# MAP AND MODEL                 #
#################################

# Town06 is ideal as it features long, straight highways with multiple lanes.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Speed units are in m/s. 
# 1 m/s = 3.6 km/h
# Ego Speed: 33 m/s ≈ 118.8 km/h
# Target Speed: 8 m/s ≈ 28.8 km/h
# Differential: 25 m/s = 90 km/h (Significantly > 50 km/h)

EGO_SPEED = 33 
TARGET_SPEED = 8
SPAWN_DISTANCE = Range(70, 100) # Give the system room to approach at high speed

EGO_MODEL = "vehicle.tesla.model3"
TARGET_MODEL = "vehicle.toyota.prius"

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

# The Ego vehicle uses a behavior that drives at the target speed but attempts to avoid collisions
behavior EgoHighSpeedBehavior(speed):
    try:
        # We use a lower avoidance threshold to simulate a late-braking or high-speed closing scenario
        do DriveAvoidingCollisions(target_speed=speed, avoidance_threshold=20)
    interrupt when withinDistanceToObjsInLane(self, 15):
        # Emergency braking if too close
        take SetBrakeAction(1.0)

behavior TargetSlowBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for long straight highway sections
highwayLanes = []
for lane in network.lanes:
    # Look for lanes that are part of the highway (long stretches)
    if 'NONE' not in lane.tags: 
        for sec in lane.sections:
            if sec.isForward:
                highwayLanes.append(sec)

# Select a random starting section
initLaneSec = Uniform(*highwayLanes)

# Define spawn point for Ego
egoSpawnPt = new OrientedPoint in initLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Target vehicle first so Ego can be placed relative to it if needed, 
# or spawn Ego and then Target ahead.
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoHighSpeedBehavior(EGO_SPEED)

target = new Car following roadDirection from ego for SPAWN_DISTANCE,
    with blueprint TARGET_MODEL,
    with behavior TargetSlowBehavior(TARGET_SPEED)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure we are not spawning directly into an intersection to keep the road straight
require (distance from ego to intersection) > 50
require (distance from target to intersection) > 50

# Ensure they are in the same lane for the closing response test
require target.laneSection == ego.laneSection

# Terminate when the ego passes the target or after a significant distance
terminate when (distance from ego to egoSpawnPt) > 250
terminate when (ego.speed < 1) and (distance from ego to target < 10) # Ego stopped