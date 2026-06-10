'''The ego-vehicle encounters an pedestrian emerging from behind a parked vehicle and advancing into the lane. The ego-vehicle must brake or maneuver to avoid it.'''
Town = 'Town03'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    initialDirection = self.heading
    while (distance to self) > globalParameters.OPT_ADV_DISTANCE:
        wait
    while True:
        take SetWalkingDirectionAction(initialDirection)
        take SetWalkingSpeedAction(globalParameters.OPT_ADV_SPEED)
        for _ in range(globalParameters.OPT_WAIT_STEP_1):
            wait
        take SetWalkingSpeedAction(0)  # Stop suddenly
        for _ in range(globalParameters.OPT_WAIT_STEP_2):
            wait

param OPT_ADV_SPEED = Range(0, 5)
param OPT_ADV_DISTANCE = Range(0, 10)
param OPT_WAIT_STEP_1 = Range(0, 30)  # Wait time in steps for the first direction
param OPT_WAIT_STEP_2 = Range(0, 30)  # Wait time in steps for the second direction
# Identifying lane sections with no adjacent lanes
laneSecsWithNoAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is None and laneSec._laneToRight is None:
            laneSecsWithNoAdjacent.append(laneSec)

# Selecting a random lane section from identified sections for the ego vehicle
egoLaneSec = Uniform(*laneSecsWithNoAdjacent)
egoSpawnPt = OrientedPoint in egoLaneSec.centerline

# Ego vehicle setup
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
# Parameters for scenario elements
param OPT_GEO_BLOCKER_Y_DISTANCE = Range(0, 40)
param OPT_GEO_X_DISTANCE = Range(-8, 0)  # Offset for the agent in the opposite lane
param OPT_GEO_Y_DISTANCE = Range(10, 30)

# Setting up the parked car that blocks the ego's path
laneSec = network.laneSectionAt(ego)  # Assuming network.laneSectionAt(ego) is predefined in the geometry part
IntSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
Blocker = Car at IntSpawnPt,
    with heading IntSpawnPt.heading,
    with regionContainedIn None

# Setup for the motorcyclist who unexpectedly enters the scene
SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Pedestrian at Blocker offset along IntSpawnPt.heading by SHIFT,
    with heading IntSpawnPt.heading + 180 deg,  # The agent is facing the opposite direction, indicating oncoming
    with regionContainedIn laneSec._laneToLeft,  # Positioned in the left lane, assuming it's the oncoming traffic lane
    with behavior AdvBehavior()