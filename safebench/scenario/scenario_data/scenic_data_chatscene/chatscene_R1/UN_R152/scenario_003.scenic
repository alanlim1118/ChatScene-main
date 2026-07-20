'''the subject vehicle drives a small radius curved road of which the guard pipes are constructed to the outer side, and a stationary vehicle (M1 category), a stationary pedestrian target or a stationary bicycle target is positioned just outside of the guard pipes and where on the extension of the centre of the lane'''
Town = 'Town03'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior WalkStraightBehavior(direction, speed):
    while True:
        take SetWalkingDirectionAction(direction)
        take SetWalkingSpeedAction(speed)

behavior AdvBehavior():
    while (distance to self) > 60:
        wait
    do WalkStraightBehavior(IntSpawnPt.heading + 180 deg, globalParameters.OPT_ADV_SPEED) until (distance to self) < globalParameters.OPT_ADV_DISTANCE
    take SetWalkingSpeedAction(0)

param OPT_ADV_DISTANCE = Range(0, 20)
param OPT_ADV_SPEED = Range(0, 5)
# Identifying lane sections with both left and right lanes moving in the same forward direction
laneSecsWithBothSides = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward == laneSec.isForward) and 
           (laneSec._laneToRight is not None and laneSec._laneToRight.isForward == laneSec.isForward):
            laneSecsWithBothSides.append(laneSec)

# Selecting a random lane section from identified sections for the ego vehicle
egoLaneSec = Uniform(*laneSecsWithBothSides)
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
AdvAgent = Car, Pedestrian, or Bicycle (stationary) at Blocker offset along IntSpawnPt.heading by SHIFT,
    with heading IntSpawnPt.heading + 180 deg,  # The agent is facing the opposite direction, indicating oncoming
    with regionContainedIn laneSec._laneToLeft,  # Positioned in the left lane, assuming it's the oncoming traffic lane
    with behavior AdvBehavior()