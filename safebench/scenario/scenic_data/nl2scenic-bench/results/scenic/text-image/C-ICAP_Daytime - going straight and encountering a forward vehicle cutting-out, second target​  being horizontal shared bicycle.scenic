"""Scenario Description:

This traffic scenario illustrates a four-way intersection with two-way, two-lane roads where a green vehicle, serving as the vehicle under test, travels straight in the right lane behind a blue vehicle. As they approach the intersection, a zebra crossing with yellow stripes is visible ahead. A stationary shared bicycle is positioned horizontally on the right side of the zebra crossing, creating a potential hazard for the vehicles proceeding straight through the junction.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
LEAD_MODEL = 'vehicle.tesla.model3'
BICYCLE_MODEL = 'vehicle.diamondback.century'

EGO_COLOR = (0, 255, 0)       # Green
LEAD_COLOR = (0, 0, 255)      # Blue

EGO_SPEED = 8
LEAD_SPEED = 7
BRAKE_INTENSITY = 0.9
SAFETY_DISTANCE = 12

EGO_OFFSET = Range(-25, -20)
LEAD_OFFSET = Range(-12, -8)
BICYCLE_LATERAL_OFFSET = 3.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        take SetBrakeAction(BRAKE_INTENSITY)

behavior LeadVehicleBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=LEAD_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersection = Uniform(*fourWayIntersections)

# Find straight maneuvers from incoming lanes
straightManeuvers = []
for lane in intersection.incomingLanes:
    for m in lane.maneuvers:
        if m.type == ManeuverType.STRAIGHT:
            straightManeuvers.append(m)

maneuver = Uniform(*straightManeuvers)

startLane = maneuver.startLane
connectingLane = maneuver.connectingLane
endLane = maneuver.endLane

ego_trajectory = [startLane, connectingLane, endLane]

# Reference point at the end of the start lane (near intersection/crosswalk)
crossingRefPoint = new OrientedPoint at startLane.centerline[-1],
    facing roadDirection

# Spawn points along the start lane centerline
egoSpawn = new OrientedPoint at crossingRefPoint
leadSpawn = new OrientedPoint at crossingRefPoint

# Bicycle placed on the right side of the zebra crossing (end lane near intersection)
bicycleRefPoint = new OrientedPoint at endLane.centerline[0],
    facing roadDirection

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (green) in right lane going straight
ego = new Car following roadDirection from egoSpawn for EGO_OFFSET,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior EgoBehavior(trajectory=ego_trajectory)

# Lead vehicle (blue) ahead of ego in same lane
lead = new Car following roadDirection from leadSpawn for LEAD_OFFSET,
    with blueprint LEAD_MODEL,
    with color LEAD_COLOR,
    with behavior LeadVehicleBehavior(trajectory=ego_trajectory)

# Stationary shared bicycle on right side of zebra crossing, oriented horizontally
bicycle = new Bicycle at bicycleRefPoint offset by BICYCLE_LATERAL_OFFSET@0,
    with blueprint BICYCLE_MODEL,
    with heading 90 deg relative to bicycleRefPoint.heading,
    with regionContainedIn None

# Ensure proper ordering: lead is ahead of ego
require distance(ego, lead) > 5
require distance(ego, intersection) <= 30
require distance(lead, intersection) <= 25

terminate when (distance to egoSpawn) > 60