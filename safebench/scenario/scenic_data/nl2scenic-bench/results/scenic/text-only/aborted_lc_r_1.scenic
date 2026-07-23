"""Scenario Description:

The ego car steers toward the right lane before veering back into its original path to perform an aborted lane change right. The adversarial car proceeds straight ahead in the adjacent right lane, acting as the lead object in this scenario.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(3, 6)
param OPT_ABORT_DISTANCE = Range(8, 15)  # Distance at which ego aborts lane change
param OPT_ADV_LEAD_DISTANCE = Range(15, 25)  # How far ahead the adv car starts in right lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    """
    Ego attempts a lane change to the right, but aborts and returns to original lane
    when getting too close to the adversarial vehicle ahead in the right lane.
    """
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_ABORT_DISTANCE):
        # Abort the lane change: steer back to original lane
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right lane (ego will be in the left lane of a pair)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

require len(laneSecsWithRightLane) > 0

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial agent in the right lane, ahead of the ego vehicle
adjLaneSec = egoLaneSec._laneToRight
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_LEAD_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup (in left lane of the pair)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Adversarial agent in the right lane, ahead of ego, driving straight
AdvAgent = new Car at advSpawnPt,
    with heading adjLaneSec.centerline.orientationAt(advSpawnPt.position),
    with regionContainedIn adjLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

# Ensure sufficient road length for the maneuver
require distance from egoSpawnPt to egoLaneSec.centerline.end >= 60