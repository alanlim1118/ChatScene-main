"""Scenario Description:

The ego vehicle navigates a curved rural road lined with trees and a pile of wood debris on the left shoulder under overcast skies. As the vehicle approaches a bend, a white passenger microvan emerges from around a blind corner, drifting entirely into the oncoming lane and cutting off the ego vehicle's path. This results in a sudden head-on collision, causing the camera view to shift abruptly towards the right side of the road, capturing the roadside vegetation and a small structure as the event concludes.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural map with curved roads and vegetation
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.domains.driving.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
MICROVAN_MODEL = "vehicle.volkswagen.t2"  # White passenger microvan blueprint
DEBRIS_MODEL = "static.prop.woodpile"     # Wood debris prop

EGO_SPEED = Range(8, 12)                  # Ego speed approaching bend (m/s)
MICROVAN_SPEED = Range(10, 14)            # Microvan speed when emerging
COLLISION_DISTANCE = 3.5                  # Distance threshold for head-on collision detection
CAMERA_SHIFT_DELAY = 0.2                  # Delay after collision before camera shifts
TRIGGER_DISTANCE = 40                     # Distance at which microvan begins to emerge
DEBRIS_OFFSET_FROM_SHOULDER = 1.5         # Offset of debris from left shoulder edge

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveBehavior():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, COLLISION_DISTANCE):
        take SetThrottleAction(0), SetBrakeAction(1)

behavior MicrovanEmergingBehavior(trigger_dist, speed):
    # Wait until ego is close enough to trigger emergence
    while distance from self to ego > trigger_dist:
        wait
    # Drive into oncoming lane (opposite direction of ego's lane)
    do FollowLaneBehavior(target_speed=speed)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Select a curved road segment suitable for blind corner scenario
curvedRoads = filter(lambda s: s.isCurved and not s.isIntersection, network.roads)
selectedRoad = Uniform(*curvedRoads)
egoLane = Uniform(*selectedRoad.lanes)

# Place ego on the curved road
egoSpawnPt = new OrientedPoint in egoLane.centerline
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior(),
    with regionContainedIn None

# Find the opposing lane for the microvan (oncoming traffic)
opposingLanes = filter(lambda l: l is not egoLane and l.road is selectedRoad, selectedRoad.lanes)
require len(list(opposingLanes)) > 0
oncomingLane = Uniform(*opposingLanes)

# Place microvan ahead around the blind corner in the oncoming lane
bendPoint = new OrientedPoint following oncomingLane.orientation from egoSpawnPt for TRIGGER_DISTANCE
microvanSpawn = new OrientedPoint at bendPoint,
    facing opposite of ego.heading
microvan = new Car at microvanSpawn,
    with blueprint MICROVAN_MODEL,
    with color "white",
    with behavior MicrovanEmergingBehavior(TRIGGER_DISTANCE, MICROVAN_SPEED),
    with regionContainedIn None

# Place wood debris on the left shoulder near the bend
leftShoulder = egoLane.leftEdge
debrisSpot = new OrientedPoint on visible leftShoulder,
    following egoLane.orientation from egoSpawnPt for Range(15, 25)
woodDebris = new Object at debrisSpot offset by (0, DEBRIS_OFFSET_FROM_SHOULDER),
    with blueprint DEBRIS_MODEL,
    with regionContainedIn None

# Environmental conditions: overcast sky
param weather = Weather(precipitation=0, cloudiness=90, wetness=20, fog=10)

# Terminate after collision or timeout
terminate when (distance from ego to microvan <= COLLISION_DISTANCE) or (elapsedTime > 30 seconds)