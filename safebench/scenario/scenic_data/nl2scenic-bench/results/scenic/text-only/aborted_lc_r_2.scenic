"""Scenario Description:

The ego car steers toward the right lane before veering back into its original path to perform an aborted lane change right. Behind it, the adversarial car moves straight ahead in the adjacent right lane, acting as the following object in this scenario.

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

param OPT_EGO_SPEED = Range(5, 8)
param OPT_ADV_SPEED = Range(5, 8)
param OPT_ABORT_DISTANCE = Range(3, 6)       # Distance at which ego aborts lane change
param OPT_FOLLOWING_DIST = Range(8, 15)      # Initial distance of adv behind ego in right lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    """Ego attempts a right lane change but aborts and returns to original lane."""
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_ABORT_DISTANCE):
        # Abort the lane change: steer back to original lane
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    terminate

behavior AdvFollowingBehavior():
    """Adversarial car follows its lane straight ahead at constant speed."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a valid right lane for the aborted lane change
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial car spawns in the right lane, behind the ego vehicle
adjRightLaneSec = egoLaneSec._laneToRight
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for -globalParameters.OPT_FOLLOWING_DIST,
    with regionContainedIn adjRightLaneSec

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn adjRightLaneSec,
    with blueprint EGO_MODEL,
    with behavior AdvFollowingBehavior()

require distance from egoSpawnPt to advSpawnPt >= globalParameters.OPT_FOLLOWING_DIST * 0.8