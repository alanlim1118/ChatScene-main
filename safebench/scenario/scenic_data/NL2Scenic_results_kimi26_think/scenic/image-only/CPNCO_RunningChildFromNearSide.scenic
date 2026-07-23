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

# Scenario geometric parameters
J = 1.00       # Lateral distance from ego centerline to obstructions (m)
E = 4.00       # Lateral distance from ego centerline to pedestrian initial position (m)
I = 1.00       # Longitudinal distance ahead of leading obstruction front to pedestrian (m)
G = 1.00       # Running distance over which pedestrian accelerates (m)

# Dynamic parameters
EGO_SPEED = 30 / 3.6       # m/s (~30 km/h typical test speed)
PED_SPEED = 12 / 3.6       # m/s (~12 km/h running speed)
TRIGGER_DIST = 15.0        # Distance at which pedestrian begins to cross (m)

EGO_COLOR = Color(1, 1, 1)         # White
OBS_COLOR = Color(0.05, 0.05, 0.05) # Black

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Drive forward at constant speed without braking
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior StationaryBehavior():
    while True:
        wait

behavior RunAcrossBehavior(impact_pt, trigger_dist):
    # Wait until ego is near the impact point, then run across
    while distance from ego to impact_pt > trigger_dist:
        wait
    take SetWalkingSpeedAction(PED_SPEED)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road lane
lane = Uniform(*network.lanes)

# Impact point L on the vehicle centerline (lane centerline)
impact_point = new OrientedPoint on lane.centerline

# Leading obstruction reference: I meters behind impact point L on centerline
obs_ref = new OrientedPoint behind impact_point by I,
    on lane.centerline

# Leading obstruction vehicle: J meters to the right (nearside) of centerline
leading_obs = new Car right of obs_ref by J,
    facing roadDirection,
    with color OBS_COLOR,
    with behavior StationaryBehavior()

# Additional stationary obstruction vehicles forming a line behind the leading one
obs2 = new Car behind leading_obs by 4,
    facing roadDirection,
    with color OBS_COLOR,
    with behavior StationaryBehavior()

obs3 = new Car behind leading_obs by 8,
    facing roadDirection,
    with color OBS_COLOR,
    with behavior StationaryBehavior()

# Pedestrian initial position: E meters laterally from centerline (right side),
# aligned longitudinally with impact point L
pedestrian_start = new OrientedPoint right of impact_point by E,
    facing -90 deg relative to roadDirection

# Ego start position: behind impact point on the centerline
ego_start = new OrientedPoint behind impact_point by 50,
    on lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at ego_start,
    facing roadDirection,
    with color EGO_COLOR,
    with behavior EgoBehavior()

pedestrian = new Pedestrian at pedestrian_start,
    with behavior RunAcrossBehavior(impact_point, TRIGGER_DIST)

terminate after 15 seconds