"""Scenario Description:

In an urban area during daylight under clear weather conditions, a traffic scenario unfolds at a driveway or alley location with a posted speed limit of 25 mph where a vehicle is backing out. The diagram illustrates this vehicle reversing and turning from the side entrance into the main roadway, indicated by curved arrows near its rear bumper. Simultaneously, another vehicle is traveling straight down the adjacent lane, marked by a downward arrow. The path of the reversing vehicle intersects with the lane of the moving traffic, resulting in a collision between the backing car and the vehicle proceeding forward on the road.

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

MODEL = 'vehicle.lincoln.mkz_2017'

# Speed limit 25 mph ≈ 11.2 m/s
BACKING_SPEED = VerifaiRange(2, 4)       # Slow reverse speed in m/s
THRU_SPEED = VerifaiRange(8, 11)         # Through traffic speed in m/s

BACKING_INIT_DIST = [5, 15]              # Distance of backing car from road edge
THRU_INIT_DIST = [20, 40]                # Distance of through car from conflict point

CRASH_DIST = 3
TERM_TIME = 30

#################################
# AGENT BEHAVIORS               #
#################################

behavior BackingOutBehavior(target_speed):
    """Vehicle reverses out of driveway/alley into the main road."""
    try:
        do ReverseWithSteeringBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior ThruTrafficBehavior(target_speed):
    """Vehicle travels straight along the main roadway."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-intersection or 3-way intersection to represent driveway/alley entry
intersection = Uniform(*filter(lambda i: i.is3Way or i.isTIntersection, network.intersections))

# The through-traffic lane is one of the main road lanes at the intersection
thruManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
thruInitLane = thruManeuver.startLane
thruTrajectory = [thruInitLane, thruManeuver.connectingLane, thruManeuver.endLane]
thruSpawnPt = new OrientedPoint in thruInitLane.centerline

# The backing vehicle starts on the minor/driveway lane entering the intersection
backingLane = Uniform(*filter(lambda l: l is not thruInitLane and l is not thruManeuver.endLane,
                               intersection.incomingLanes))
backingSpawnRegion = Region.fromShapely(backingLane.geometry.buffer(0))
backingSpawnPt = new OrientedPoint in backingSpawnRegion,
    facing toward intersection.center

#################################
# SCENARIO SPECIFICATION        #
#################################

# Environmental conditions: daylight, clear weather
param time_of_day = 'noon'
param weather_preset = 'ClearNoon'

# Backing vehicle (ego) - reversing out of driveway
ego = new Car at backingSpawnPt,
    with blueprint MODEL,
    with behavior BackingOutBehavior(BACKING_SPEED)

# Through-traffic vehicle (adversary) - traveling straight on main road
adversary = new Car at thruSpawnPt,
    with blueprint MODEL,
    with behavior ThruTrafficBehavior(THRU_SPEED)

# Spatial constraints to ensure proper initial positioning
require BACKING_INIT_DIST[0] <= (distance from ego to intersection.center) <= BACKING_INIT_DIST[1]
require THRU_INIT_DIST[0] <= (distance from adversary to intersection.center) <= THRU_INIT_DIST[1]

# Ensure the backing vehicle's path can intersect with through traffic
require (distance from ego to thruInitLane) < 10

# Termination conditions
terminate when simulation().currentTime > TERM_TIME
terminate when withinDistanceToAnyObjs(ego, CRASH_DIST)