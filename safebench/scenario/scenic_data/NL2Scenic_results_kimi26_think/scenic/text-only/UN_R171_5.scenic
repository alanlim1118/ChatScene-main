"""Scenario Description:

The ego vehicle attempts to initiate a lane change while a parallel passenger car maintains a steady position directly in the "blind spot" or adjacent zone, requiring the ego vehicle to maintain its lane and wait for the adjacent vehicle to either accelerate or fall back before executing the lateral shift.

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

param OPT_EGO_SPEED = Range(5, 10)                # Ego and initial blind-spot car speed (m/s)
param OPT_STEADY_TIME = Range(3, 6)               # Seconds the blind-spot car stays parallel
param OPT_SPEED_DELTA = Uniform(3, 5)             # Speed change when the adjacent car accelerates or falls back
param OPT_ADJ_ACTION = Uniform(0, 1)              # 0 = fall back, 1 = accelerate
param OPT_BLIND_SPOT_DISTANCE = 7.0                 # Distance threshold to abort/avoid lane change (meters)
param OPT_RETRY_INTERVAL = 1.0                      # Seconds to wait before retrying lane change
param OPT_EGO_WAIT_TIME = Range(2, 4)             # Seconds ego drives before attempting lane change

#################################
# AGENT BEHAVIORS               #
#################################

behavior BlindSpotBehavior():
    """
    Maintain a steady position alongside the ego, then either accelerate
    or fall back to open the gap for the lane change.
    """
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) for globalParameters.OPT_STEADY_TIME seconds
    if globalParameters.OPT_ADJ_ACTION == 0:
        # Fall back
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED - globalParameters.OPT_SPEED_DELTA)
    else:
        # Accelerate away
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED + globalParameters.OPT_SPEED_DELTA)

behavior EgoBehavior():
    """
    Drive along the current lane, attempt a lane change to the left,
    and abort/retry if the blind-spot car is still too close.
    """
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) for globalParameters.OPT_EGO_WAIT_TIME seconds
    
    while True:
        try:
            do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
            break  # Lane change completed successfully
        interrupt when (distance from self to BlindSpotCar < globalParameters.OPT_BLIND_SPOT_DISTANCE):
            # Blind spot still occupied; maintain lane and wait
            do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) for globalParameters.OPT_RETRY_INTERVAL seconds

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

adjLaneSec = egoLaneSec._laneToLeft
BlindSpotSpawnPt = adjLaneSec.centerline.project(egoSpawnPt.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Blind-spot car setup: directly adjacent in the left lane
BlindSpotCar = new Car at BlindSpotSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with behavior BlindSpotBehavior()

require distance to intersection > 100