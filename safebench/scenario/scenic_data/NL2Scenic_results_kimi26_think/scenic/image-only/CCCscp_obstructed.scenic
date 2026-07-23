"""Scenario Description:

At a four-way intersection, two lanes of queued black vehicles are positioned on the left side of the approach. The upper lane contains three cars spaced 1 meter apart with the lead vehicle 21 meters from the intersection, while the lower lane has three similar cars spaced 1 meter apart, positioned 24 meters away. Between these queues, a white ego vehicle is positioned in a central lane, aligned with a horizontal path indicating forward motion straight across the junction. Perpendicular to this, a crossing white vehicle travels from the top of the intersection downwards along a vertical path. The scenario results in a collision where the frontal structure of the test vehicle strikes the side of the crossing vehicle.

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

# Queued black vehicle parameters
UPPER_LEAD_DIST = -21
LOWER_LEAD_DIST = -24
QUEUE_SPACING = -1

# Ego and adversary parameters
param EGO_DIST = Uniform(25, 35)
param ADV_DIST = Uniform(5, 15)
param EGO_SPEED = VerifaiRange(8, 12)
param ADV_SPEED = VerifaiRange(4, 8)
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select a central incoming lane that has adjacent lanes on both sides
egoInitLane = Uniform(*filter(lambda l: l.left is not None and l.right is not None, intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adjacent lanes for the queued vehicles
upperLane = egoInitLane.left
lowerLane = egoInitLane.right

# Conflicting straight maneuver for the crossing adversary
advManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points at the intersection entry
egoSpawn = egoInitLane.centerline[-1]
upperSpawn = upperLane.centerline[-1]
lowerSpawn = lowerLane.centerline[-1]
advSpawn = advManeuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Upper lane: 3 black cars, 1m apart, lead vehicle 21m from intersection
upperCar1 = new Car following roadDirection from upperSpawn for UPPER_LEAD_DIST,
    with color (0, 0, 0)
upperCar2 = new Car following roadDirection from upperSpawn for UPPER_LEAD_DIST + QUEUE_SPACING,
    with color (0, 0, 0)
upperCar3 = new Car following roadDirection from upperSpawn for UPPER_LEAD_DIST + 2*QUEUE_SPACING,
    with color (0, 0, 0)

# Lower lane: 3 black cars, 1m apart, lead vehicle 24m from intersection
lowerCar1 = new Car following roadDirection from lowerSpawn for LOWER_LEAD_DIST,
    with color (0, 0, 0)
lowerCar2 = new Car following roadDirection from lowerSpawn for LOWER_LEAD_DIST + QUEUE_SPACING,
    with color (0, 0, 0)
lowerCar3 = new Car following roadDirection from lowerSpawn for LOWER_LEAD_DIST + 2*QUEUE_SPACING,
    with color (0, 0, 0)

# Ego vehicle: white, central lane, straight across intersection
ego = new Car following roadDirection from egoSpawn for globalParameters.EGO_DIST,
    with color (255, 255, 255),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

# Adversary vehicle: white, crossing top-to-bottom
adversary = new Car following roadDirection from advSpawn for globalParameters.ADV_DIST,
    with color (255, 255, 255),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

terminate when (distance to egoSpawn) > TERM_DIST