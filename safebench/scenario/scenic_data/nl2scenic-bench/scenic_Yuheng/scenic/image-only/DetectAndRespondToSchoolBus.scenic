"""Scenario Description:

The ego vehicle (green autonomous car) is driving on a straight, undivided multilane highway in the right lane. A school bus is stopped in the opposing left lane facing the opposite direction with red stop arms extended and lights activated to allow students to disembark. The ego vehicle approaches the stopped school bus and must respond appropriately to the school bus signals.

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
BUS_MODEL = "vehicle.ford.school_bus"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BRAKE_DIST = Range(15, 25)
param OPT_BUS_DISTANCE_AHEAD = Range(40, 60)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        terminate

behavior StoppedBusBehavior():
    """School bus remains stopped with stop arms extended."""
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight road segment with at least 2 lanes (one per direction)
straightRoads = filter(lambda r: len(r.lanes) >= 2 and r.isStraight, network.roads)
road = Uniform(*straightRoads)

# Ego travels in the rightmost lane going forward
egoLane = Uniform(*filter(lambda l: l.isForward, road.lanes))
egoSpawnPt = new OrientedPoint in egoLane.centerline

# School bus is in the opposing lane, positioned ahead of ego but facing opposite direction
opposingLanes = filter(lambda l: not l.isForward and l.road is road, road.lanes)
busLane = Uniform(*opposingLanes)

# Place bus ahead of ego along the road, but in the opposing lane
busPositionAlongRoad = new OrientedPoint ahead of egoSpawnPt by globalParameters.OPT_BUS_DISTANCE_AHEAD
busSpawnPt = new OrientedPoint in busLane.centerline,
    offset laterally by 0,
    with heading busLane.centerline.headingAt(busPositionAlongRoad.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(),
    with color (0, 255, 0)  # Green autonomous vehicle

schoolBus = new Car at busSpawnPt,
    with regionContainedIn None,
    with blueprint BUS_MODEL,
    with behavior StoppedBusBehavior(),
    with color (255, 255, 255)  # White school bus

# Ensure the bus is actually in the opposing lane and properly oriented
require busLane.road is egoLane.road
require abs(egoSpawnPt.heading - busSpawnPt.heading) > 150 deg  # Facing roughly opposite directions
require 30 <= (distance from egoSpawnPt to busSpawnPt) <= 70