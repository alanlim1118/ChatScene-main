"""Scenario Description:

The ego vehicle is traveling straight on a road at a steady speed of 50 km/h. An adult pedestrian enters from the left (nearside) and walks across the vehicle's path. The ego vehicle maintains its course and speed without braking, resulting in a collision where the front of the vehicle impacts the pedestrian crossing the road.

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

PEDESTRIAN_WALKING_SPEED = 1.4  # Typical adult walking speed in m/s
PEDESTRIAN_START_OFFSET = 8     # Lateral offset from lane center for pedestrian start (left/nearside)
PEDESTRIAN_TRIGGER_DISTANCE = 25  # Distance ahead of ego when pedestrian starts crossing

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantSpeedBehavior(target_speed):
    """Maintain constant speed without braking."""
    while True:
        take SetTargetSpeedAction(target_speed)

behavior CrossFromLeftBehavior(trigger_distance, walking_speed):
    """Wait until ego is within trigger distance, then cross perpendicular to road."""
    while distance from self to ego > trigger_distance:
        wait
    take SetWalkingDirectionAction(self.heading)
    take SetWalkingSpeedAction(walking_speed)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight section of road
straightLane = Uniform(*filter(lambda l: l.maneuvers and any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), network.lanes))
egoSpawnPt = new OrientedPoint in straightLane.centerline

# Define pedestrian spawn point on the left (nearside) of the ego's future path
pedestrianSpawnPt = new OrientedPoint following straightLane.orientation from egoSpawnPt for PEDESTRIAN_TRIGGER_DISTANCE,
    with heading straightLane.heading - 90 deg  # Facing right (across the road from left side)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior ConstantSpeedBehavior(EGO_SPEED_MS)

pedestrian = new Pedestrian left of pedestrianSpawnPt by PEDESTRIAN_START_OFFSET,
    with heading pedestrianSpawnPt.heading,
    with regionContainedIn None,
    with behavior CrossFromLeftBehavior(PEDESTRIAN_TRIGGER_DISTANCE, PEDESTRIAN_WALKING_SPEED)

require distance from ego to pedestrian > PEDESTRIAN_TRIGGER_DISTANCE - 5

terminate when (distance from ego to pedestrian < 2) or (simulation().currentTime > 30)