"""Scenario Description:

At night on a multi-lane highway illuminated by streetlights, the ego vehicle drives in the center lane while following a white car ahead. A large yellow and green coach bus rapidly approaches from the right lane and overtakes the ego vehicle with extremely minimal side clearance, creating a close-call situation as it passes. The bus then merges left into the ego vehicle's lane directly in front, cutting in closely before stabilizing its position. The bus continues forward in the lane, joining the flow of traffic which includes other vehicles and a truck visible further ahead under green overhead highway signs.

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
BUS_MODEL = "vehicle.volkswagen.t2"
TRUCK_MODEL = "vehicle.carlamotors.firetruck"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BUS_SPEED = Range(16, 22)
param OPT_LEAD_CAR_SPEED = Range(8, 12)
param OPT_TRAFFIC_SPEED = Range(8, 14)
param OPT_BUS_MERGE_DISTANCE = Range(5, 10)
param OPT_CLOSE_PASS_LATERAL = Range(0.3, 0.8)
param OPT_CUT_IN_DISTANCE = Range(8, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoFollowBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, 10)):
        take SetThrottleAction(0)
        take SetBrakeAction(0.8)
        do WaitBehavior() for 3 seconds
        terminate

behavior LeadCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_CAR_SPEED)

behavior BusOvertakeAndMergeBehavior(ego_ref, merge_dist, cut_in_dist):
    # Phase 1: Overtake in right lane at high speed
    do FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED) until (
        (distance along roadDirection from self to ego_ref) < -merge_dist
    )
    # Phase 2: Merge left into ego's lane
    do LaneChangeBehavior(direction='left', target_speed=globalParameters.OPT_BUS_SPEED) until (
        self.lane is ego_ref.lane
    )
    # Phase 3: Stabilize and continue in lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED)

behavior TrafficBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable highway section with multiple lanes
highwaySection = Uniform(*filter(
    lambda s: len(s.lanes) >= 3 and s.road.isHighway,
    network.roadSections
))

# Define lanes: left, center (ego), right (bus)
sortedLanes = sorted(highwaySection.lanes, key=lambda l: l.laneID)
centerLaneIdx = len(sortedLanes) // 2
egoLane = sortedLanes[centerLaneIdx]
rightLane = sortedLanes[centerLaneIdx + 1] if centerLaneIdx + 1 < len(sortedLanes) else sortedLanes[-1]

# Spawn points along the highway
egoSpawnPt = new OrientedPoint in egoLane.centerline
leadCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(25, 35)
busSpawnPt = new OrientedPoint in rightLane.centerline offset laterally by globalParameters.OPT_CLOSE_PASS_LATERAL
busSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(-30, -20) in rightLane.centerline

truckSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(80, 120)
trafficSpawnPt1 = new OrientedPoint following roadDirection from egoSpawnPt for Range(50, 70) in rightLane.centerline
trafficSpawnPt2 = new OrientedPoint following roadDirection from egoSpawnPt for Range(60, 90) in egoLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Night weather with streetlights
param weather = 'ClearSunset'

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoFollowBehavior()

# White lead car in same lane
leadCar = new Car at leadCarSpawnPt,
    with regionContainedIn None,
    with color Color.withBytes([255, 255, 255]),
    with behavior LeadCarBehavior()

# Yellow-green coach bus in right lane
bus = new Car at busSpawnPt,
    with regionContainedIn None,
    with blueprint BUS_MODEL,
    with color Color.withBytes([200, 200, 0]),
    with behavior BusOvertakeAndMergeBehavior(ego, globalParameters.OPT_BUS_MERGE_DISTANCE, globalParameters.OPT_CUT_IN_DISTANCE)

# Truck further ahead
truck = new Car at truckSpawnPt,
    with regionContainedIn None,
    with blueprint TRUCK_MODEL,
    with behavior TrafficBehavior(globalParameters.OPT_TRAFFIC_SPEED)

# Additional traffic vehicles
trafficCar1 = new Car at trafficSpawnPt1,
    with regionContainedIn None,
    with behavior TrafficBehavior(globalParameters.OPT_TRAFFIC_SPEED)

trafficCar2 = new Car at trafficSpawnPt2,
    with regionContainedIn None,
    with behavior TrafficBehavior(globalParameters.OPT_TRAFFIC_SPEED)

# Ensure proper spacing constraints
require distance from ego to leadCar >= 20
require distance from bus to ego >= 15
require bus.lane is not ego.lane initially
require truck.lane is ego.lane or truck.lane is rightLane