"""Scenario Description:

The scenario takes place on a long straight road with three lanes separated by white dashed lines, where a blue vehicle labeled VUT is driving in the middle lane behind a red vehicle labeled VT. The red vehicle is shown cutting out urgently to the left lane, indicated by a curved red arrow, while further ahead in the right lane, a half-squatting child and an orange dog are positioned as obstacles.

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

VUT_MODEL = 'vehicle.lincoln.mkz_2017'
VT_MODEL = 'vehicle.tesla.model3'
CHILD_MODEL = 'walker.pedestrian.0014'
DOG_MODEL = 'walker.pedestrian.0014'  # Using pedestrian blueprint as dog proxy; color set to orange

param OPT_VUT_SPEED = Range(8, 12)
param OPT_VT_SPEED = Range(10, 14)
param OPT_VT_CUTOUT_DISTANCE = Range(15, 25)  # Distance ahead of VUT when VT begins cutout
param OPT_OBSTACLE_DISTANCE = Range(40, 60)  # Distance ahead of VUT where obstacles are placed
param OPT_LANE_OFFSET = 3.5  # Approximate lane width for lateral positioning

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior CutOutLeftBehavior(target_speed, cutout_distance):
    """VT drives forward then urgently cuts to the left lane."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to ego <= cutout_distance):
        take SetTurnSignalAction(-1)  # Left turn signal
        do ChangeLaneBehavior(direction=-1, target_speed=target_speed)
        take SetTurnSignalAction(0)
        do FollowLaneBehavior(target_speed=target_speed)

behavior EgoFollowBehavior(target_speed):
    """VUT follows its lane at a constant speed."""
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment with at least 3 lanes
straightRoad = Uniform(*filter(lambda r: len(r.lanes) >= 3 and r.isStraight, network.roads))
middleLane = straightRoad.lanes[1]  # Middle lane (0-indexed: left=0, middle=1, right=2)
leftLane = straightRoad.lanes[0]
rightLane = straightRoad.lanes[2]

# Spawn points along the middle lane centerline
vutSpawnPt = new OrientedPoint in middleLane.centerline
vtSpawnPt = new OrientedPoint following middleLane.orientation from vutSpawnPt for Range(20, 30)
obstacleSpawnPt = new OrientedPoint following middleLane.orientation from vutSpawnPt for globalParameters.OPT_OBSTACLE_DISTANCE

# Lateral offsets for obstacles in the right lane
childOffsetPt = new OrientedPoint at obstacleSpawnPt offset by (globalParameters.OPT_LANE_OFFSET, 0)
dogOffsetPt = new OrientedPoint at obstacleSpawnPt offset by (globalParameters.OPT_LANE_OFFSET + 1.5, 0)

#################################
# SCENARIO SPECIFICATION        #
#################################

# VUT (blue vehicle) in the middle lane
ego = new Car at vutSpawnPt,
    with blueprint VUT_MODEL,
    with color Color.withBytes([0, 100, 255]),
    with regionContainedIn middleLane,
    with behavior EgoFollowBehavior(globalParameters.OPT_VUT_SPEED)

# VT (red vehicle) ahead of VUT in the middle lane, will cut out left
VT = new Car at vtSpawnPt,
    with blueprint VT_MODEL,
    with color Color.withBytes([220, 20, 20]),
    with regionContainedIn middleLane,
    with behavior CutOutLeftBehavior(globalParameters.OPT_VT_SPEED, globalParameters.OPT_VT_CUTOUT_DISTANCE)

# Half-squatting child in the right lane ahead
child = new Pedestrian at childOffsetPt,
    with blueprint CHILD_MODEL,
    with heading middleLane.orientation,
    with regionContainedIn rightLane,
    with behavior WaitBehavior()

# Orange dog in the right lane ahead, slightly offset from child
dog = new Pedestrian at dogOffsetPt,
    with blueprint DOG_MODEL,
    with color Color.withBytes([255, 165, 0]),
    with heading middleLane.orientation,
    with regionContainedIn rightLane,
    with behavior WaitBehavior()

# Ensure the road segment is long enough for the scenario
require length(straightRoad) > 100

terminate when distance from ego to vtSpawnPt > 80