"""Scenario Description:

In this traffic scenario, a white vehicle under test travels forward along its longitudinal centerline axis (BB) towards a child pedestrian dummy running across its path from the right side. The pedestrian emerges from behind a line of stationary black obstruction vehicles (axis CC) positioned on the nearside, with the test vehicle maintaining a lateral distance of 1.00 m (J) from these obstructions. The pedestrian dummy, initially positioned 4.00 m (E) laterally from the test vehicle's centerline, accelerates over a running distance of 1.00 m (G) and is located 1.00 m (I) ahead of the front of the leading obstruction vehicle. The pedestrian's trajectory (axis AA) intersects the test vehicle's centerline at point L, which marks the impact position for 50% overlap scenarios. The scenario depicts a collision where the frontal structure of the moving vehicle strikes the pedestrian without any braking action being applied.

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
OBSTRUCTION_MODEL = "vehicle.tesla.model3"
PEDESTRIAN_MODEL = "walker.pedestrian.child"

LATERAL_DIST_J = 1.00       # Lateral distance from ego to obstruction vehicles
LATERAL_OFFSET_E = 4.00     # Pedestrian initial lateral offset from ego centerline
RUNNING_DIST_G = 1.00       # Distance pedestrian accelerates before reaching ego path
AHEAD_OFFSET_I = 1.00       # Pedestrian ahead of leading obstruction vehicle front
EGO_SPEED = 8.0             # Ego constant speed (no braking)
PED_WALK_SPEED = 1.8        # Child pedestrian running speed
PED_TRIGGER_DIST = 6.0      # Distance at which pedestrian starts crossing

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantSpeedNoBrake(target_speed):
    """Ego drives at constant speed without any braking."""
    do FollowLaneBehavior(target_speed=target_speed)

behavior ChildCrossFromRight(ego_ref, trigger_dist, walk_speed):
    """Pedestrian waits until ego is close, then runs across from right to left."""
    while distance from self to ego_ref > trigger_dist:
        wait
    # Cross perpendicular to ego heading (from right side)
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(walk_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight lane segment for the scenario
straightLane = Uniform(*filter(lambda l: l.isStraight and len(l.centerline) > 60, network.lanes))
egoSpawnPt = new OrientedPoint in straightLane.centerline

# Place ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (1.0, 1.0, 1.0),
    with behavior ConstantSpeedNoBrake(EGO_SPEED)

# Leading obstruction vehicle placed to the right of ego's path
leadingObstruction = new Car right of egoSpawnPt by LATERAL_DIST_J,
    with blueprint OBSTRUCTION_MODEL,
    with color (0.0, 0.0, 0.0),
    with regionContainedIn None

# Additional obstruction vehicles behind the leading one to form a line
obstruction2 = new Car behind leadingObstruction by 5.0,
    with blueprint OBSTRUCTION_MODEL,
    with color (0.0, 0.0, 0.0),
    with regionContainedIn None

obstruction3 = new Car behind obstruction2 by 5.0,
    with blueprint OBSTRUCTION_MODEL,
    with color (0.0, 0.0, 0.0),
    with regionContainedIn None

# Pedestrian spawn point: ahead of leading obstruction by I, and laterally offset E from ego centerline
# The pedestrian starts further right (behind obstructions) and crosses toward ego path
pedSpawnBase = new OrientedPoint ahead of leadingObstruction by AHEAD_OFFSET_I
pedSpawnPt = new OrientedPoint right of pedSpawnBase by (LATERAL_OFFSET_E - LATERAL_DIST_J),
    facing -90 deg relative to ego.heading

# Child pedestrian dummy
childPedestrian = new Pedestrian at pedSpawnPt,
    with blueprint PEDESTRIAN_MODEL,
    with regionContainedIn None,
    with behavior ChildCrossFromRight(ego, PED_TRIGGER_DIST, PED_WALK_SPEED)

#################################
# REQUIREMENTS & TERMINATION    #
#################################

require distance from ego to leadingObstruction > 30
require distance from ego to childPedestrian > 20

terminate after 30 seconds