"""Scenario Description:

In a top-down view of a two-lane road, a blue ego vehicle executes a lane change maneuver, steering from the left lane into the right lane as indicated by a curved blue trajectory arrow. This scenario is described as a lane change from oncoming traffic with a leading vehicle. Two pink adversarial vehicles are present: one is positioned in the right lane ahead of the ego vehicle, traveling in the same direction as indicated by a right-pointing arrow, serving as the leading vehicle, while the other pink vehicle is located in the left lane with a left-pointing arrow, representing oncoming traffic approaching in the lane the ego vehicle is departing.

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

param OPT_EGO_SPEED = Range(4, 6)
param OPT_LEADING_SPEED = Range(3, 5)
param OPT_ONCOMING_SPEED = Range(5, 8)
param OPT_LEADING_DIST = Range(25, 40)
param OPT_ONCOMING_DIST = Range(30, 50)
param OPT_BRAKE_DIST = Range(5, 7)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to LeadingAgent < globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
    terminate

behavior OncomingBehavior():
    while True:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ONCOMING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right lane (ego starts in left lane, changes to right)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

require len(laneSecsWithRightLane) > 0

egoLaneSec = Uniform(*laneSecsWithRightLane)
targetLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Leading vehicle spawn point: ahead of ego in the right (target) lane
rightLanePt = targetLaneSec.centerline.project(egoSpawnPt.position)
LeadingSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_LEADING_DIST

# Oncoming vehicle spawn point: ahead of ego in the left (current) lane, facing opposite direction
# The oncoming lane is typically the _laneToLeft of the ego's current lane if it exists,
# but per description the oncoming traffic is in the same left lane the ego is departing.
# We place it in the ego's lane but with reversed heading to simulate oncoming traffic.
OncomingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ONCOMING_DIST,
    with heading egoSpawnPt.heading + 3.14159  # Opposite direction

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in the left lane, changing to the right lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),  # Blue
    with behavior EgoBehavior()

# Leading adversarial vehicle in the right lane, ahead of ego, same direction
LeadingAgent = new Car at LeadingSpawnPt,
    with heading LeadingSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),  # Pink
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

# Oncoming adversarial vehicle in the left lane, approaching from ahead, opposite direction
OncomingAgent = new Car at OncomingSpawnPt,
    with heading OncomingSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),  # Pink
    with behavior OncomingBehavior()

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150