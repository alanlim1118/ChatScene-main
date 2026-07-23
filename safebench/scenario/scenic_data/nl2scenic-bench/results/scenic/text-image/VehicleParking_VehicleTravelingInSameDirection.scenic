"""Scenario Description:

In an urban area during daylight with clear weather and a posted speed limit of 25 mph, a vehicle is depicted leaving a parked position by angling out from the curb into the traffic lane. The scene includes a designated handicapped parking spot marked with a yellow wheelchair symbol along the curb, and as the vehicle pulls out, it encounters another vehicle traveling in the same direction in a non-junction area.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.domains.driving.model

#################################
# CONSTANTS                     #
#################################

SPEED_LIMIT_MPH = 25
SPEED_LIMIT_MS = SPEED_LIMIT_MPH * 0.44704  # Convert mph to m/s
PULL_OUT_ANGLE = 30 deg                      # Angle at which ego angles out from curb
ONCOMING_DISTANCE_MIN = 20                   # Minimum initial distance for oncoming vehicle
ONCOMING_DISTANCE_MAX = 40                   # Maximum initial distance for oncoming vehicle
CURB_OFFSET = 0.5                            # Lateral offset from curb for parked position

#################################
# AGENT BEHAVIORS               #
#################################

behavior PullOutFromCurb(target_speed):
    """Ego starts parked, then steers into lane and accelerates."""
    take SetThrottleAction(0), SetBrakeAction(1)
    wait
    # Begin pulling out: steer left (into traffic) and accelerate
    do FollowLaneBehavior(target_speed)

behavior DriveStraight(target_speed):
    """Oncoming vehicle drives straight in its lane at target speed."""
    try:
        do FollowLaneBehavior(target_speed)
    interrupt when self.speed < 0.5:
        take SetThrottleAction(0), SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-junction road segment with a curb (urban area)
nonJunctionRoads = [r for r in network.roads if not r.isIntersection]
selectedRoad = Uniform(*nonJunctionRoads)
selectedLane = Uniform(*selectedRoad.lanes)
curb = selectedLane.group.curb

# Define handicapped parking spot location along the curb
handicapSpot = new OrientedPoint on visible curb

# Ego spawn point: parked at curb near handicap spot, angled slightly toward traffic
egoSpawn = new OrientedPoint at handicapSpot offset laterally by CURB_OFFSET,
    facing handicapSpot.heading + PULL_OUT_ANGLE

# Oncoming vehicle spawn: ahead of ego in the same lane/direction
oncomingSpawn = new OrientedPoint following selectedLane.centerline from egoSpawn for Range(ONCOMING_DISTANCE_MIN, ONCOMING_DISTANCE_MAX)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather and time parameters
param weather = 'ClearNoon'
param timeOfDay = 12

# Ego vehicle: pulling out from handicapped parking spot
ego = new Car at egoSpawn,
    with behavior PullOutFromCurb(SPEED_LIMIT_MS),
    with regionContainedIn None

# Handicapped parking spot marker (static object representing yellow wheelchair symbol)
new Object at handicapSpot,
    with blueprint 'static.prop.warning_sign',
    with regionContainedIn None

# Oncoming vehicle traveling in same direction in non-junction area
oncomingCar = new Car at oncomingSpawn,
    with behavior DriveStraight(SPEED_LIMIT_MS),
    with regionContainedIn None

# Ensure we are in a non-junction area
require not ego.isOnIntersection

# Ensure oncoming car is in same lane group and direction as ego
require oncomingCar.laneGroup is ego.laneGroup

terminate after 30 seconds