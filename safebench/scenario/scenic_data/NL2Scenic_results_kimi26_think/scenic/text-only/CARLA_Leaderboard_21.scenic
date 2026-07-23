"""Scenario Description:

While performing a maneuver, the ego-vehicle encounters an obstacle in the road, either a pedestrian or a bicycle, and must perform an emergency brake or an avoidance maneuver.

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

EGO_SPEED = 10
EGO_BRAKING_THRESHOLD = 12

BRAKE_ACTION = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

# Stationary behavior for the obstacle
behavior Stationary():
    while True:
        wait

# EGO BEHAVIOR: Follow lane, and brake or avoid when encountering the obstacle
behavior EgoBehavior(speed=10):
    try:
        do FollowLaneBehavior(speed)
    interrupt when withinDistanceToAnyObjs(self, EGO_BRAKING_THRESHOLD):
        if (egoLaneSec._laneToLeft is not None):
            do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, target_speed=speed)
        else:
            take SetBrakeAction(BRAKE_ACTION)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane that has a left lane available for avoidance
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)

# Spawn point for the obstacle in the ego's lane
obstacleSpawnPt = new OrientedPoint on egoLaneSec.centerline

# Ego spawn point behind the obstacle
egoSpawnPt = new OrientedPoint following roadDirection from obstacleSpawnPt for Range(-40, -25)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Obstacle: either a pedestrian or a bicycle in the road
obstacle = new Uniform(Pedestrian, Bicycle) at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn None,
    with behavior Stationary()

# Ego vehicle approaching the obstacle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

require (distance to intersection) > 75
terminate when ego.speed < 0.1 and (distance to obstacle) < 15