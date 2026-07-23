"""Scenario Description:

The ego vehicle travels forward in the left lane of a busy urban road beneath a large elevated concrete structure during the late afternoon, maintaining a speed of approximately 55 km/h. To the right, commercial buildings and heavy traffic are visible. Suddenly, a white passenger sedan from the far right lanes aggressively cuts across two lanes of traffic, disregarding the flow of vehicles. The white car swerves directly into the ego vehicle's path, resulting in a sudden side-impact collision as the ego vehicle is unable to avoid the intruding vehicle.

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
ADV_MODEL = "vehicle.tesla.model3"
ADV_COLOR = "255,255,255"  # White

EGO_SPEED_KMH = 55
EGO_SPEED_MS = EGO_SPEED_KMH / 3.6  # Convert to m/s (~15.28 m/s)

ADV_SPEED_MS = EGO_SPEED_MS * 1.1  # Slightly faster for aggressive cut-in
CUT_IN_TRIGGER_DIST = Range(40, 55)  # Distance at which adv begins cutting in
LANE_CHANGE_DURATION = Range(1.5, 2.5)  # Aggressive lane change time

TRAFFIC_SPEED_MS = EGO_SPEED_MS * Uniform(0.7, 0.9)
TRAFFIC_AHEAD_DIST = Range(25, 40)
TRAFFIC_RIGHT_DIST = Range(15, 30)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveBehavior():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED_MS)
    interrupt when (distance from self to AdvCar < 5):
        take SetThrottleAction(0), SetBrakeAction(1)

behavior AggressiveCutInBehavior(trigger_dist, target_speed):
    # Wait until ego is within trigger distance
    while distance from self to ego > trigger_dist:
        do FollowLaneBehavior(target_speed=target_speed)
    # Aggressively cut left across lanes toward ego's path
    do LaneChangeBehavior(direction='left', target_speed=target_speed)
    # Continue in new lane after cut-in
    do FollowLaneBehavior(target_speed=target_speed)

behavior TrafficFollowBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a road section with at least 3 forward lanes (left + middle + right)
multiLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToRight is not None and sec._laneToRight._laneToRight is not None:
            multiLaneSections.append(sec)

require len(multiLaneSections) > 0
egoLaneSec = Uniform(*multiLaneSections)
middleLaneSec = egoLaneSec._laneToRight
rightLaneSec = middleLaneSec._laneToRight

# Ego spawn point in left lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial car spawn point in far right lane, ahead of ego
advProjOnRight = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advProjOnRight for Range(30, 50)

# Background traffic in middle lane
midProjOnMiddle = middleLaneSec.centerline.project(egoSpawnPt.position)
trafficMidSpawn = new OrientedPoint following roadDirection from midProjOnMiddle for TRAFFIC_AHEAD_DIST

# Background traffic in right lane (behind adv car)
trafficRightProj = rightLaneSec.centerline.project(egoSpawnPt.position)
trafficRightSpawn = new OrientedPoint following roadDirection from trafficRightProj for TRAFFIC_RIGHT_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in left lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior()

# Adversarial white sedan in far right lane
AdvCar = new Car at advSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint ADV_MODEL,
    with color ADV_COLOR,
    with behavior AggressiveCutInBehavior(CUT_IN_TRIGGER_DIST, ADV_SPEED_MS)

# Background traffic - middle lane
TrafficMid = new Car at trafficMidSpawn,
    with regionContainedIn middleLaneSec,
    with behavior TrafficFollowBehavior(TRAFFIC_SPEED_MS)

# Background traffic - right lane
TrafficRight = new Car at trafficRightSpawn,
    with regionContainedIn rightLaneSec,
    with behavior TrafficFollowBehavior(TRAFFIC_SPEED_MS)

# Ensure scenario runs long enough for the cut-in and collision
terminate after 30 seconds