"""Scenario Description:

The ego vehicle attempts to initiate a lane change while a parallel passenger car maintains a steady position directly in the blind spot or adjacent zone, requiring the ego vehicle to maintain its lane and wait for the adjacent vehicle to either accelerate or fall back before executing the lateral shift.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(8, 12)          # Ego cruising speed in m/s
param OPT_ADV_SPEED_INITIAL = Range(8, 12)  # Adversary initial speed matching ego
param OPT_ADV_WAIT_TIME = Range(3, 6)       # Time adversary stays in blind spot
param OPT_ADV_SPEED_AFTER = Range(14, 18)   # Adversary accelerates away after waiting
param OPT_BLIND_SPOT_OFFSET = Range(-2, 2)  # Longitudinal offset for blind spot positioning
param OPT_LC_CLEARANCE_DIST = Range(15, 25) # Min distance to adjacent car before lane change allowed
param OPT_EGO_TARGET_SPEED = Range(10, 14)  # Ego speed during/after lane change

#################################
# AGENT BEHAVIORS               #
#################################

behavior BlindSpotAdversaryBehavior():
    """Adversary maintains speed alongside ego (blind spot), then accelerates away."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED_INITIAL) for globalParameters.OPT_ADV_WAIT_TIME seconds
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED_AFTER)

behavior EgoBlindSpotBehavior():
    """Ego follows lane, waits until adjacent lane is clear, then changes lane."""
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdjCar > globalParameters.OPT_LC_CLEARANCE_DIST):
        do LaneChangeBehavior(
            laneSectionToSwitch=egoLaneSec._laneToLeft,
            is_oppositeTraffic=False,
            target_speed=globalParameters.OPT_EGO_TARGET_SPEED
        )
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_TARGET_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find forward lane sections that have a forward left lane (same-direction adjacent)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversary in the left adjacent lane at roughly the same longitudinal position
adjLaneSec = egoLaneSec._laneToLeft
adjBasePt = adjLaneSec.centerline.project(egoSpawnPt.position)
adjSpawnPt = new OrientedPoint following roadDirection from adjBasePt for globalParameters.OPT_BLIND_SPOT_OFFSET

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBlindSpotBehavior()

# Adjacent car in blind spot
AdjCar = new Car at adjSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with behavior BlindSpotAdversaryBehavior()

# Ensure scenario starts far enough from intersections for meaningful lane change
require distance to intersection >= 150