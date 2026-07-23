"""Scenario Description:

The ego-vehicle must exit a parallel parking bay into a flow of traffic. The ego starts parked on the right side of the road and must wait for a safe gap in oncoming traffic before merging into the driving lane. An adversary vehicle approaches from behind in the target lane, requiring the ego to time its departure correctly to avoid a collision.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(5, 9)
param OPT_ADV_START_DIST = Range(40, 70)  # Distance behind ego where adversary spawns
param OPT_SAFE_MERGE_GAP = Range(15, 25)  # Minimum distance to adv required to start merging
param OPT_PARKING_OFFSET = Range(2.5, 3.5)  # Lateral offset from lane center for parked position

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoExitParkingBehavior():
    # Wait until adversary is far enough away to safely merge
    do StandStillBehavior() until (distance from self to AdvAgent > globalParameters.OPT_SAFE_MERGE_GAP or AdvAgent is None)
    # Merge into the driving lane
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    # Continue driving in the lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvApproachBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right neighbor (parking lane / shoulder)
laneSecsWithRightNeighbor = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToRight is not None:
            laneSecsWithRightNeighbor.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightNeighbor)
parkingLaneSec = egoLaneSec._laneToRight

# Ego spawn point in the parking lane
egoSpawnPt = new OrientedPoint in parkingLaneSec.centerline

# Adversary spawn point behind ego in the driving lane
AdvSpawnBase = new OrientedPoint following roadDirection from egoSpawnPt for -globalParameters.OPT_ADV_START_DIST
AdvSpawnPt = egoLaneSec.centerline.project(AdvSpawnBase.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle starts parked on the right side of the road
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn parkingLaneSec,
    with behavior EgoExitParkingBehavior()

# Adversary vehicle approaching from behind in the driving lane
AdvAgent = new Car at AdvSpawnPt,
    with heading egoLaneSec.centerline.headingAt(AdvSpawnPt),
    with regionContainedIn egoLaneSec,
    with blueprint "vehicle.tesla.model3",
    with behavior AdvApproachBehavior()

require distance to intersection > 80