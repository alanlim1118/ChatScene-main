"""Scenario Description:

This simulated traffic scenario presents a top-down view of a roadway where an ego vehicle, depicted as a central white rectangular block with black vertical stripes, travels forward at a steady speed of 50 km/h. Telemetry data at the bottom of the screen confirms the vehicle's constant velocity and increasing distance traveled. A pedestrian, represented by a thin vertical yellow line, enters the frame from the left side, corresponding to the farside of the road, and runs perpendicularly across the vehicle's path. The ego vehicle continues its forward motion without applying any brakes, maintaining a direct course toward the crossing pedestrian. As the sequence progresses, the pedestrian moves closer to the center of the lane, placing themselves directly in the path of the vehicle's front end, culminating in a collision where the vehicle strikes the pedestrian.

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
EGO_SPEED_KMH = 50
EGO_SPEED_MS = EGO_SPEED_KMH / 3.6  # Convert km/h to m/s

PED_SPEED = Range(1.4, 2.0)          # Running speed for pedestrian (m/s)
CROSSING_TRIGGER_DIST = Range(18, 25) # Distance ahead when pedestrian starts crossing
PEDESTRIAN_START_OFFSET = Range(4, 6) # Lateral offset from lane centerline (farside/left)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoConstantSpeedBehavior(target_speed):
    """Ego drives at constant speed without braking."""
    while True:
        take SetTargetSpeedAction(target_speed)

behavior PedestrianCrossFromLeftBehavior(ego_ref, trigger_dist, walk_speed):
    """Pedestrian waits until ego is within trigger distance, then crosses perpendicular from left."""
    while distance from self to ego_ref > trigger_dist:
        wait
    take SetWalkingSpeedAction(walk_speed)
    # Walk perpendicular to road (toward right side from left/farside)
    do CrossingBehavior(ego_ref, walk_speed, 20)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment for the scenario
straightRoads = filter(lambda r: len(r.sections) == 1 and not r.isIntersection, network.roads)
selectedRoad = Uniform(*straightRoads)
egoLane = Uniform(*selectedRoad.lanes)

# Define spawn point for ego on the lane centerline
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Define pedestrian start position: ahead of ego and offset to the left (farside)
pedAheadDist = Range(30, 45)
pedStartBase = new OrientedPoint following egoLane.orientation from egoSpawnPt for pedAheadDist
pedStartPt = new OrientedPoint left of pedStartBase by PEDESTRIAN_START_OFFSET,
    with heading pedStartBase.heading + 90 deg  # Facing perpendicular across the road (left to right)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EgoConstantSpeedBehavior(EGO_SPEED_MS)

pedestrian = new Pedestrian at pedStartPt,
    with regionContainedIn None,
    with behavior PedestrianCrossFromLeftBehavior(ego, CROSSING_TRIGGER_DIST, PED_SPEED)

# Ensure sufficient distance for the scenario to play out
require distance from ego to pedestrian >= 25

terminate after 20 seconds