"""Scenario Description:

The ego car travels straight forward within the middle lane. Meanwhile, an adversarial object cuts ahead by turning from the left lane toward the right.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_ADV_CUT_DIST = Range(15, 25)  # Distance ahead of ego where adv starts cutting
param OPT_BRAKE_DIST = Range(5, 8)     # Emergency brake distance for ego

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
    terminate

behavior CutInBehavior(target_speed=10):
    """
    Adversarial behavior: drive in left lane, then cut across to the right lane
    by following a trajectory that crosses through the ego's middle lane.
    """
    # Phase 1: Follow left lane until close enough to cut
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego) < globalParameters.OPT_ADV_CUT_DIST
    
    # Phase 2: Execute lane change from left to right (crossing middle lane)
    # The adversarial car is in the left lane; _laneToRight is the middle lane,
    # and middle._laneToRight is the right lane. We perform two sequential lane changes.
    middleLane = self.lane._laneToRight
    if middleLane is not None and middleLane._laneToRight is not None:
        do LaneChangeBehavior(laneSectionToSwitch=middleLane, is_oppositeTraffic=False, target_speed=target_speed)
        do LaneChangeBehavior(laneSectionToSwitch=middleLane._laneToRight, is_oppositeTraffic=False, target_speed=target_speed)
    
    # Phase 3: Continue in right lane
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left and right neighbor (i.e., middle lanes)
laneSecsWithBothNeighbors = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            laneSecsWithBothNeighbors.append(laneSec)

require len(laneSecsWithBothNeighbors) > 0

egoLaneSec = Uniform(*laneSecsWithBothNeighbors)
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point in middle lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial spawn point in left lane, slightly behind or alongside ego
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnOffset = Range(-5, 5)  # Slight longitudinal offset
advSpawnPt = new OrientedPoint following roadDirection from leftLanePt for advSpawnOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with behavior CutInBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

require distance from egoSpawnPt to nearest intersection >= 80
terminate when distance from ego to AdvAgent > 60