"""Scenario Description:

The ego vehicle drives straight on a wet, multi-lane city street under overcast conditions, passing shops and trees on both sides. A white SUV is visible in the right adjacent lane, moving slightly faster than the ego vehicle. Suddenly, the white SUV swerves sharply to the left, cutting directly into the ego vehicle's lane to avoid another vehicle merging from its right. This abrupt lane change causes a sudden side-impact collision, forcing the ego vehicle to decelerate rapidly as the white SUV crosses its path and moves toward the center of the road.

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
SUV_MODEL = "vehicle.tesla.modely"
MERGER_MODEL = "vehicle.nissan.patrol"

param OPT_EGO_SPEED = Range(6, 9)
param OPT_SUV_SPEED = globalParameters.OPT_EGO_SPEED + Range(1.5, 3.0)
param OPT_MERGER_SPEED = globalParameters.OPT_EGO_SPEED - Range(1, 2)

param OPT_SUV_LATERAL_DIST = Range(8, 15)       # Initial lateral offset of SUV ahead of ego
param OPT_MERGER_TRIGGER_DIST = Range(18, 25)   # Distance at which SUV initiates lane change
param OPT_EGO_BRAKE_DIST = Range(4, 8)          # Distance at which ego brakes hard
param OPT_EGO_BRAKE_AMOUNT = 1.0                # Full emergency brake

WEATHER_PRESET = 'WetCloudyNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to SuvAgent < brake_dist):
        take SetBrakeAction(brake_amount)
        do WaitBehavior() for 5 seconds
        terminate

behavior SuvBehavior(suv_speed, trigger_dist, target_lane):
    do FollowLaneBehavior(target_speed=suv_speed) until (distance from self to MergerAgent < trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=suv_speed)
    do FollowLaneBehavior(target_speed=suv_speed)

behavior MergerBehavior(merger_speed):
    do FollowLaneBehavior(target_speed=merger_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left and right adjacent forward lane
laneSecsWithBothNeighbors = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward
            and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None and laneSec._laneToRight.isForward):
            laneSecsWithBothNeighbors.append(laneSec)

require len(laneSecsWithBothNeighbors) > 0

egoLaneSec = Uniform(*laneSecsWithBothNeighbors)
rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft  # Not used directly but confirms multi-lane context

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place SUV in right adjacent lane, slightly ahead of ego
rightLaneProj = rightLaneSec.centerline.project(egoSpawnPt.position)
suvSpawnPt = new OrientedPoint following roadDirection from rightLaneProj for globalParameters.OPT_SUV_LATERAL_DIST

# Place merger vehicle further ahead in the right-right lane or same right lane ahead of SUV
# Since we need a vehicle merging from the right of the SUV, place it ahead in the right lane
mergerSpawnPt = new OrientedPoint following roadDirection from suvSpawnPt for Range(15, 25)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set weather to wet overcast
mutate weather to WEATHER_PRESET

# Ego vehicle in center lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_BRAKE_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# White SUV in right adjacent lane, moving faster
SuvAgent = new Car at suvSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint SUV_MODEL,
    with color (1.0, 1.0, 1.0),
    with behavior SuvBehavior(
        globalParameters.OPT_SUV_SPEED,
        globalParameters.OPT_MERGER_TRIGGER_DIST,
        egoLaneSec
    )

# Merger vehicle ahead of SUV in right lane, triggering the SUV's evasive maneuver
MergerAgent = new Car at mergerSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint MERGER_MODEL,
    with behavior MergerBehavior(globalParameters.OPT_MERGER_SPEED)

require distance to intersection >= 100
terminate when (distance from ego to egoSpawnPt) > 120