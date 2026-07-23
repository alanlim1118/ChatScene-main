"""Scenario Description:

The ego vehicle travels forward through a wide urban intersection during the day with a green traffic light visible overhead. A black SUV abruptly cuts into the ego vehicle's lane from the left, seemingly maneuvering to avoid a hazard or make a turn, forcing the ego vehicle into a sudden emergency braking maneuver. The black SUV passes directly in front of the ego vehicle, resulting in a side-swipe collision with the front right side of the ego car. Following the collision, the black SUV continues to the right side of the road, and a forklift carrying a large load is visible stationary on the sidewalk to the right near the storefronts.

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
SUV_MODEL = "vehicle.nissan.patrol"
FORKLIFT_MODEL = "vehicle.tesla.cybertruck"  # Placeholder; CARLA may not have a forklift blueprint

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SUV_SPEED = Range(10, 14)
param OPT_BRAKE_TRIGGER_DIST = Range(12, 18)
param OPT_CUT_IN_DIST = Range(20, 30)       # Distance ahead of ego where SUV begins cut-in
param OPT_SUV_CONTINUE_DIST = Range(30, 50) # Distance SUV continues after passing ego

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBrakeBehavior(target_speed, brake_trigger_dist):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to SuvAgent < brake_trigger_dist):
        take SetThrottleAction(0), SetBrakeAction(1)
        do WaitBehavior() for 10 seconds
        terminate

behavior SuvCutInBehavior(target_speed, cut_in_dist, continue_dist):
    # Wait until close enough to ego to initiate cut-in
    while distance from self to ego > cut_in_dist:
        do FollowLaneBehavior(target_speed=target_speed)
    # Cut into ego's lane from the left
    do LaneChangeBehavior(laneSectionToSwitch=ego.laneSection, target_speed=target_speed)
    # Continue driving in ego's lane briefly to cause side-swipe
    do FollowLaneBehavior(target_speed=target_speed) for 3 seconds
    # After collision/passing, move to right side of road
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to ego > continue_dist):
        take SetThrottleAction(0), SetBrakeAction(0.3)
        do WaitBehavior() for 10 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find an intersection with at least two forward lanes (left lane for SUV, right/center for ego)
wideIntersections = []
for intersection in network.intersections:
    if intersection.is4Way or intersection.is3Way:
        incomingRoads = intersection.incomingRoads
        for road in incomingRoads:
            forwardLanes = [sec for sec in road.sections if sec.isForward]
            if len(forwardLanes) >= 2:
                wideIntersections.append((intersection, road, forwardLanes))
                break

require len(wideIntersections) > 0
selectedIntersection, selectedRoad, forwardLanes = Uniform(*wideIntersections)

# Ego spawns in the rightmost forward lane approaching the intersection
egoLaneSec = forwardLanes[-1]
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# SUV spawns in the left forward lane, behind ego initially so it can catch up and cut in
suvLaneSec = forwardLanes[0]
suvSpawnOffset = Range(-40, -25)  # Behind ego
suvBasePt = suvLaneSec.centerline.project(egoSpawnPt.position)
suvSpawnPt = new OrientedPoint following roadDirection from suvBasePt for suvSpawnOffset

# Forklift on the right sidewalk near storefronts past the intersection
rightSidewalk = None
for section in selectedRoad.sections:
    if section._sidewalkRight is not None:
        rightSidewalk = section._sidewalkRight
        break

if rightSidewalk is None:
    # Fallback: place forklift offset to the right of the road end
    forkliftSpawnPt = new OrientedPoint right of egoSpawnPt by 8,
        with heading egoSpawnPt.heading
else:
    forkliftBasePt = new OrientedPoint in rightSidewalk.centerline
    forkliftSpawnPt = new OrientedPoint at forkliftBasePt,
        with heading forkliftBasePt.heading + 90 deg  # Facing perpendicular toward road

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ensure daytime and green light conditions via parameters
param timeOfDay = 'day'
param trafficLightState = 'green'

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBrakeBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_TRIGGER_DIST)

SuvAgent = new Car at suvSpawnPt,
    with regionContainedIn suvLaneSec,
    with blueprint SUV_MODEL,
    with color 'black',
    with behavior SuvCutInBehavior(globalParameters.OPT_SUV_SPEED, globalParameters.OPT_CUT_IN_DIST, globalParameters.OPT_SUV_CONTINUE_DIST)

forklift = new Car at forkliftSpawnPt,
    with regionContainedIn None,
    with blueprint FORKLIFT_MODEL,
    with behavior WaitBehavior()

# Require ego starts far enough from intersection for scenario to play out
require distance from ego to selectedIntersection >= 40

terminate after 45 seconds