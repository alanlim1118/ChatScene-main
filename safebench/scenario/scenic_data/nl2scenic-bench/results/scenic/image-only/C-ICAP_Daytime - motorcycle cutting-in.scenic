"""Scenario Description:

The scenario takes place on a long, straight road with two lanes separated by a dotted line, where a green vehicle travels in the upper lane and a motorcycle travels in the lower lane. Both vehicles are moving in the same direction, with the motorcycle positioned ahead of the green vehicle in the adjacent lane. As the green vehicle approaches the motorcycle, the motorcycle initiates a lane change maneuver, curving from the lower lane into the upper lane, thereby cutting into the lane occupied by the green vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
MOTO_MODEL = "vehicle.kawasaki.ninja"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_MOTO_SPEED = Range(6, 10)
param OPT_INITIAL_GAP = Range(30, 50)
param OPT_LANE_CHANGE_TRIGGER_DIST = Range(15, 25)
param OPT_BRAKE_DIST = Range(8, 15)

GREEN_COLOR = (0, 200, 0)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoFollowLaneBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior MotorcycleLaneChangeBehavior(speed, trigger_distance, ego_ref):
    # Initially follow the current lane
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego_ref <= trigger_distance)
    # Initiate lane change to the left (upper lane)
    do LaneChangeBehavior(direction=Left, target_speed=speed)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight road section with at least two lanes going in the same direction
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 2 and s.isStraight, network.roadSegments))
upperLane = roadSegment.lanes[0]   # Upper lane (leftmost in driving direction)
lowerLane = roadSegment.lanes[1]   # Lower lane (rightmost / adjacent)

# Spawn points along the respective lane centerlines
egoSpawnPt = new OrientedPoint in upperLane.centerline
motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_INITIAL_GAP,
    offset laterally by -upperLane.width  # Place in the lower (adjacent) lane

# Project motorcycle spawn onto lower lane centerline for proper alignment
projectedMotoPt = lowerLane.centerline.project(motoSpawnPt.position)
motoHeading = lowerLane.orientation[projectedMotoPt]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color GREEN_COLOR,
    with behavior EgoFollowLaneBehavior(globalParameters.OPT_EGO_SPEED)

motorcycle = new Motorcycle at projectedMotoPt,
    with heading motoHeading,
    with regionContainedIn None,
    with blueprint MOTO_MODEL,
    with behavior MotorcycleLaneChangeBehavior(
        globalParameters.OPT_MOTO_SPEED,
        globalParameters.OPT_LANE_CHANGE_TRIGGER_DIST,
        ego
    )

# Ensure both agents are on the same road segment and properly ordered
require distance from ego to motorcycle >= globalParameters.OPT_INITIAL_GAP - 5
require distance from ego to motorcycle <= globalParameters.OPT_INITIAL_GAP + 10