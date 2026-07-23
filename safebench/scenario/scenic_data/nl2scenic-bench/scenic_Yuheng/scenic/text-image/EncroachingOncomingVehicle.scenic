"""Scenario Description:

In this top-down schematic labeled "Encroaching, Oncoming Vehicle Test Scenario," a subject vehicle outlined in green travels straight along a highway lane bordered by a dashed white line on the right and a solid yellow line on the left. An oncoming vehicle, depicted with a white outline in the opposing lane to the left of the yellow line, is shown angling sharply across the solid yellow divider, effectively drifting into the subject vehicle's lane of travel. This visual representation corresponds to a test case where an ADS-equipped vehicle must detect and react to an opposing vehicle that has lost its lane position and is encroaching into the subject vehicle's path, creating a potential head-on collision scenario.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(8, 12)
param OPT_BRAKE_DISTANCE = Range(15, 25)
param OPT_ENCROACH_ANGLE = Range(15, 35)  # Degrees the adv vehicle angles toward ego's lane
param OPT_ENCROACH_START_DIST = Range(40, 60)  # Distance ahead of ego when adv starts encroaching

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior EncroachBehavior(encroach_angle, target_speed):
    """
    Adversarial vehicle drives in its own lane initially, then steers sharply
    across the centerline into the oncoming (ego) lane at a specified angle.
    """
    # First follow own lane normally until trigger distance
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego <= globalParameters.OPT_ENCROACH_START_DIST)
    
    # Now encroach: steer toward ego's lane at the specified angle
    # We use a constant steering action to simulate loss of lane position
    # The heading offset creates the angled drift across the yellow line
    while True:
        # Steer toward the ego vehicle's lane (right from adv's perspective since it's oncoming)
        take SetSteeringAction(0.6)  # Hard right steer to cross into oncoming lane
        take SetThrottleAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Find road sections that have opposing lanes (suitable for head-on scenarios)
opposingLaneSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section._laneToLeft is not None and section._laneToLeft.isForward != section.isForward:
            opposingLaneSections.append(section)

require len(opposingLaneSections) > 0

egoLaneSec = Uniform(*opposingLaneSections)
advLaneSec = egoLaneSec._laneToLeft

# Spawn ego in its lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Spawn adversarial vehicle in the opposing lane, ahead of ego
# Project a point far ahead along ego's lane, then place adv in opposing lane at that longitudinal position
aheadPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(80, 120)
advCenterlinePt = advLaneSec.centerline.project(aheadPt.position)
advSpawnPt = new OrientedPoint at advCenterlinePt,
    with heading advLaneSec.centerline.headingAt(advCenterlinePt)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior EncroachBehavior(globalParameters.OPT_ENCROACH_ANGLE, globalParameters.OPT_ADV_SPEED)

# Ensure sufficient distance for the scenario to play out
require distance from egoSpawnPt to advSpawnPt >= 60
require distance to intersection >= 100