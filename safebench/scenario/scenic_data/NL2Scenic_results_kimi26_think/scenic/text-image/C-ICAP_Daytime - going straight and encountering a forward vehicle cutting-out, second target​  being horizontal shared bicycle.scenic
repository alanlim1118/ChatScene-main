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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_OFFSET = -1 * Range(10, 15)
LEAD_OFFSET = -1 * Range(2, 4)
BICYCLE_LATERAL_OFFSET = Range(1.5, 2.5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=10, trajectory=trajectory)

behavior LeadBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=10, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select a straight maneuver through the intersection (right lane for through traffic)
straight_maneuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, intersection.maneuvers))
startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane
trajectory = [startLane, connectingLane, endLane]

# End of the start lane marks the intersection entrance
intersection_edge = startLane.centerline[-1]

# Spawn points for the vehicles (negative offset places them before the intersection)
leadSpawnPt = new OrientedPoint at intersection_edge
egoSpawnPt = new OrientedPoint at intersection_edge

# Point for the bicycle on the right side of the zebra crossing
zebraPoint = new OrientedPoint at intersection_edge,
    facing roadDirection

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue lead vehicle traveling straight
lead = new Car following roadDirection from leadSpawnPt for LEAD_OFFSET,
    with color (0, 0, 255),
    with behavior LeadBehavior(trajectory)

# Green ego vehicle (vehicle under test) traveling straight behind the lead
ego = new Car following roadDirection from egoSpawnPt for EGO_OFFSET,
    with blueprint EGO_MODEL,
    with color (0, 255, 0),
    with behavior EgoBehavior(trajectory)

# Stationary shared bicycle positioned horizontally on the right side of the zebra crossing
bicycle = new Bicycle at zebraPoint offset by 0 @ (-BICYCLE_LATERAL_OFFSET),
    with heading 90 deg relative to zebraPoint.heading,
    with regionContainedIn None

# Requirements to ensure proper scenario setup
require (distance from ego to lead) > 5
require (distance from lead to intersection) < 10
require (distance from bicycle to intersection) < 5
terminate when (distance to intersection) > 60