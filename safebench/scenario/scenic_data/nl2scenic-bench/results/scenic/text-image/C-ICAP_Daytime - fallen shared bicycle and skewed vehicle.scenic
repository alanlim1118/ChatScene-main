"""Scenario Description:

The scenario takes place on a straight, two-lane road divided by a white dashed center line, where a green vehicle under test travels in the left lane. Directly ahead in the right lane lies a fallen shared bicycle. Five meters further down the road in front of the bicycle, a stationary blue vehicle is positioned at a 30-degree tilt relative to the direction of travel, encroaching 0.2 meters into the left lane occupied by the approaching green vehicle.

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
EGO_COLOR = (0, 255, 0)  # Green

BLUE_CAR_MODEL = "vehicle.tesla.model3"
BLUE_CAR_COLOR = (0, 0, 255)  # Blue

BICYCLE_MODEL = "vehicle.diamondback.century"

TILT_ANGLE = 30 deg
ENCROACHMENT = 0.2
DISTANCE_BIKE_TO_BLUE_CAR = 5.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed=8):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyCars(car=self, thresholdDistance=10):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight road segment with at least 2 lanes
straightRoads = filter(lambda r: len(r.lanes) >= 2 and not r.isIntersection, network.roads)
road = Uniform(*straightRoads)

# Left lane for ego, right lane for obstacles
leftLane = road.lanes[0]
rightLane = road.lanes[1]

# Spawn point for ego in the left lane
egoSpot = new OrientedPoint in leftLane.centerline

# Reference point in the right lane ahead of ego for the fallen bicycle
bicycleSpot = new OrientedPoint in rightLane.centerline,
    ahead of egoSpot by Range(20, 30),
    facing roadDirection

# Blue car positioned 5m ahead of the bicycle in the right lane,
# but shifted 0.2m toward the left lane (encroachment)
blueCarBaseSpot = new OrientedPoint in rightLane.centerline,
    ahead of bicycleSpot by DISTANCE_BIKE_TO_BLUE_CAR,
    facing roadDirection

# Shift the blue car 0.2m to the left (toward left lane) from the right lane centerline
# In CARLA/Scenic convention, left offset is positive relative to heading
blueCarSpot = new OrientedPoint at blueCarBaseSpot offset by ENCROACHMENT@90,
    facing (roadDirection at blueCarBaseSpot) + TILT_ANGLE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpot,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior EgoBehavior(speed=8)

fallen_bicycle = new Bicycle at bicycleSpot,
    with blueprint BICYCLE_MODEL,
    with heading (roadDirection at bicycleSpot) + 90 deg,  # Fallen sideways
    with regionContainedIn None

blue_car = new Car at blueCarSpot,
    with blueprint BLUE_CAR_MODEL,
    with color BLUE_CAR_COLOR,
    with behavior StationaryBehavior(),
    with regionContainedIn None

require distance from ego to fallen_bicycle >= 15
require distance from fallen_bicycle to blue_car <= DISTANCE_BIKE_TO_BLUE_CAR + 0.5

terminate when (distance from ego to blueCarSpot) > 60