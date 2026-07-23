"""Scenario Description:

In a top-down simulation view, the ego vehicle, depicted as a red rectangle, travels forward along a grey road marked with dashed white lane lines. The ego vehicle is following its lane and approaching a white and black striped vehicle located to its right, which moves from a blue area onto the main road surface, appearing to drive in the same lane or merge into the ego vehicle's path. Simultaneously, another white and black striped vehicle is visible in the lane to the left, moving parallel to the ego vehicle. The scene captures the ego vehicle navigating through traffic, closing the distance to the vehicle on the right as it proceeds down the road.

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
MERGING_VEHICLE_MODEL = "vehicle.tesla.model3"
LEFT_VEHICLE_MODEL = "vehicle.tesla.model3"

EGO_SPEED = Range(8, 12)
MERGE_SPEED = Range(6, 10)
LEFT_VEHICLE_SPEED = Range(8, 12)

MERGE_TRIGGER_DISTANCE = Range(30, 50)

#################################
# BEHAVIORS                     #
#################################

behavior EgoFollowLane():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

behavior MergeFromRightBehavior():
    # Initially wait off-road / in merging area until ego is close enough
    while (distance from self to ego) > MERGE_TRIGGER_DISTANCE:
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        wait
    # Then merge into ego's lane by following the road
    do FollowLaneBehavior(target_speed=MERGE_SPEED)

behavior LeftParallelBehavior():
    do FollowLaneBehavior(target_speed=LEFT_VEHICLE_SPEED)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Place ego on a straight road segment
egoLane = Uniform(*filter(lambda l: l.isDrivingLane and not l.isIntersection, network.lanes))
egoSpawn = new OrientedPoint in egoLane.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoFollowLane()

# Merging vehicle starts to the right of ego, slightly ahead, representing
# a vehicle entering from a side area / ramp merging into ego's lane
rightLaneCandidates = filter(lambda l: l.isDrivingLane and l is not egoLane, network.lanes)
mergingVehicle = new Car at ego offset by Range(3, 5) @ Range(15, 30),
    with blueprint MERGING_VEHICLE_MODEL,
    with behavior MergeFromRightBehavior(),
    with regionContainedIn None

# Left parallel vehicle in the lane to the left of ego, moving alongside
leftVehicle = new Car at ego offset by Range(-5, -3) @ Range(-5, 10),
    with blueprint LEFT_VEHICLE_MODEL,
    with behavior LeftParallelBehavior(),
    with regionContainedIn None

# Ensure spatial configuration matches description
require (distance from ego to mergingVehicle) < 60
require (distance from ego to leftVehicle) < 40
require mergingVehicle is not leftVehicle