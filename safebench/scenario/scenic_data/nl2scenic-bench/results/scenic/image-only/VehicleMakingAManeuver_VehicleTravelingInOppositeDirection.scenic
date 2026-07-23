"""Scenario Description:

In a rural area during daylight under clear weather conditions, a vehicle traveling in the right lane attempts to pass a vehicle ahead of it by crossing the dashed center line at a non-junction location with a speed limit of 55 mph or more. As the passing vehicle moves into the opposing lane to overtake, it encroaches upon the path of a third vehicle traveling in the opposite direction in that left lane, creating a dangerous head-on conflict scenario.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_MODEL = "vehicle.tesla.model3"
ONCOMING_MODEL = "vehicle.audi.a2"

# Speeds in m/s (55 mph ≈ 24.6 m/s)
param EGO_SPEED = Range(24, 28)
param LEAD_SPEED = Range(18, 22)
param ONCOMING_SPEED = Range(24, 28)

PASS_START_DIST = Range(30, 50)
ONCOMING_INIT_DIST = Range(80, 120)
TERM_DIST = 200

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior OncomingBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior PassingBehavior(target_speed, lead_vehicle, pass_start_dist):
    # Follow lane behind lead vehicle until close enough to pass
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to lead_vehicle <= pass_start_dist)
    # Change lane to left (opposing) lane to overtake
    do ChangeLaneBehavior(left=True, target_speed=target_speed) for 3 seconds
    # Continue in opposing lane (creating conflict)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a road segment that is not part of any intersection and has sufficient length
nonJunctionRoads = filter(lambda r: not r.isIntersection and len(r.lanes) >= 2, network.roads)
road = Uniform(*nonJunctionRoads)

# Right lane for ego and lead vehicle
rightLane = Uniform(*filter(lambda l: l.isForward, road.lanes))
# Left (opposing) lane for oncoming vehicle
leftLane = Uniform(*filter(lambda l: not l.isForward and l.road is road, network.lanes))

# Spawn points along the right lane centerline
egoSpawnPt = new OrientedPoint in rightLane.centerline
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(15, 25)

# Oncoming vehicle spawn point in the opposing lane, ahead of ego position
oncomingSpawnPt = new OrientedPoint in leftLane.centerline,
    with heading leftLane.centerline.start.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior PassingBehavior(globalParameters.EGO_SPEED, leadVehicle, PASS_START_DIST),
    with regionContainedIn None

leadVehicle = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with behavior LeadBehavior(globalParameters.LEAD_SPEED),
    with regionContainedIn None

oncomingVehicle = new Car at oncomingSpawnPt,
    with blueprint ONCOMING_MODEL,
    with behavior OncomingBehavior(globalParameters.ONCOMING_SPEED),
    with regionContainedIn None

# Ensure oncoming vehicle is at appropriate distance when ego begins passing
require ONCOMING_INIT_DIST[0] <= (distance from ego to oncomingVehicle) <= ONCOMING_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST