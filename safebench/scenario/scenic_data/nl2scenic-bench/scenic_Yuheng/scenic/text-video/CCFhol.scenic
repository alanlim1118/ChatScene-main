"""Scenario Description:

The ego vehicle travels steadily in its lane at a constant speed of 70 km/h on a straight multi-lane road. An oncoming vehicle in the adjacent opposite-direction lane intentionally moves into the ego's lane to attempt an overtaking maneuver, resulting in a head-on collision with the ego vehicle which maintains its straight path.

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
ADV_MODEL = "vehicle.tesla.model3"

# 70 km/h ≈ 19.44 m/s
param EGO_SPEED = 19.44
param ADV_SPEED = Range(15, 20)          # Oncoming vehicle speed before lane change
param LANE_CHANGE_TRIGGER_DIST = Range(40, 60)  # Distance from ego when adv starts lane change
param CRASH_TERMINATE_DIST = 3.0         # Distance threshold to terminate after collision

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoStraightBehavior(target_speed):
    """Ego maintains constant speed in its lane without deviation."""
    do FollowLaneBehavior(target_speed=target_speed)

behavior OncomingOvertakeBehavior(target_speed, trigger_distance):
    """
    Oncoming vehicle drives in opposite lane, then changes into ego's lane
    to simulate an overtaking maneuver that causes head-on collision.
    """
    try:
        # Drive in the opposite-direction lane toward ego
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego < trigger_distance)
        # Change into ego's lane (which is to the right from oncoming perspective)
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToRight, target_speed=target_speed)
        # Continue driving in ego's lane (now heading toward ego)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, CRASH_TERMINATE_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find forward lane sections that have an adjacent opposite-direction lane to the left
# (i.e., ego drives forward, oncoming traffic is in the lane to the left going backward)
forwardLanesWithOpposing = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
            forwardLanesWithOpposing.append(laneSec)

require len(forwardLanesWithOpposing) > 0

egoLaneSec = Uniform(*forwardLanesWithOpposing)
opposingLaneSec = egoLaneSec._laneToLeft

# Place ego somewhere along its forward lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place oncoming vehicle ahead of ego in the opposing lane
# Project ego position onto opposing lane centerline, then offset forward (in opposing direction)
opposingProjPt = opposingLaneSec.centerline.project(egoSpawnPt.position)
advSpawnOffset = Range(80, 120)  # Initial distance between ego and adversary
advSpawnPt = new OrientedPoint following roadDirection from opposingProjPt for advSpawnOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle: white/black car traveling straight at 70 km/h
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (1, 1, 1),
    with behavior EgoStraightBehavior(globalParameters.EGO_SPEED)

# Adversarial oncoming vehicle: red car that changes into ego's lane
adversary = new Car at advSpawnPt,
    with regionContainedIn opposingLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0, 0),
    with behavior OncomingOvertakeBehavior(
        globalParameters.ADV_SPEED,
        globalParameters.LANE_CHANGE_TRIGGER_DIST
    )

# Ensure sufficient road length for the scenario
require distance from egoSpawnPt to intersection >= 100

# Terminate simulation after collision occurs
terminate when withinDistanceToAnyObjs(ego, CRASH_TERMINATE_DIST)