"""Scenario Description:

The ego vehicle travels on a multi-lane roadway on a clear morning, approaching an underpass beneath a concrete overpass. A large cargo truck carrying a load of white panels is driving in the adjacent right lane. As the ego vehicle proceeds forward, attempting to overtake the truck near the entrance of the underpass, the large truck unexpectedly drifts to the left. This movement causes the truck to invade the ego vehicle's lane, resulting in a sudden side-impact collision between the two vehicles as they pass under the bridge.

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
TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_TRUCK_SPEED = Range(6, 9)
param OPT_OVERTAKE_TRIGGER_DIST = Range(30, 50)   # Distance at which ego begins overtaking maneuver
param OPT_DRIFT_TRIGGER_DIST = Range(15, 25)      # Distance from ego to truck when truck starts drifting
param OPT_DRIFT_LATERAL_SPEED = Range(1.5, 3.0)   # Lateral drift speed of truck (m/s)
param OPT_INITIAL_LONG_OFFSET = Range(10, 20)     # Truck starts ahead of ego
param TERM_DISTANCE = 120                          # Termination distance from spawn

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoOvertakeBehavior():
    """Ego follows its lane, then attempts to overtake by changing to the left lane."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (
        distance from self to Truck < globalParameters.OPT_OVERTAKE_TRIGGER_DIST
    )
    # Attempt lane change to left to overtake
    try:
        do LaneChangeBehavior(
            laneSectionToSwitch=egoLaneSec._laneToLeft,
            is_oppositeTraffic=False,
            target_speed=globalParameters.OPT_EGO_SPEED
        )
    interrupt when True:
        # If lane change fails or is interrupted, continue following current lane
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior TruckDriftBehavior():
    """Truck follows its lane normally, then drifts left into ego's lane when ego is nearby."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED) until (
        distance from self to ego < globalParameters.OPT_DRIFT_TRIGGER_DIST
    )
    # Drift left: apply constant lateral velocity toward the left lane
    while True:
        take SetThrottleAction(0.3), SetSteerAction(-0.15)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a valid left neighbor (for overtaking) and a right neighbor (for truck)
validLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward
            and laneSec._laneToLeft is not None
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None
            and laneSec._laneToRight.isForward
        ):
            validLaneSections.append(laneSec)

egoLaneSec = Uniform(*validLaneSections)
rightLaneSec = egoLaneSec._laneToRight

# Ego spawn point on the center of its lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Truck spawn point: projected onto right lane, offset ahead of ego
rightLaneProj = rightLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnBase = new OrientedPoint following roadDirection from rightLaneProj for globalParameters.OPT_INITIAL_LONG_OFFSET

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoOvertakeBehavior()

Truck = new Car at truckSpawnBase,
    with heading truckSpawnBase.heading,
    with regionContainedIn rightLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior TruckDriftBehavior()

# Ensure we are far enough from intersections for a clean highway/underpass scenario
require distance to intersection >= 80

terminate when (distance from ego to egoSpawnPt > globalParameters.TERM_DISTANCE)