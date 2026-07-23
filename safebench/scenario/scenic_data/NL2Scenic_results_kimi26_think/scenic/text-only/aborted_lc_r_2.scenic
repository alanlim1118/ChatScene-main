"""Scenario Description:

The ego car steers toward the right lane before veering back into its original path to perform an aborted lane change right. Behind it, the adversarial car moves straight ahead in the adjacent right lane, acting as the following object in this scenario.

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

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_ADV_BEHIND_DIST = Range(10, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Attempt lane change to the right lane
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    # Abort the maneuver and veer back into the original lane
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    # Continue driving in the original lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify lane sections with an adjacent right lane
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
advLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place the adversarial car behind the ego in the adjacent right lane
advSpawnPt = new OrientedPoint at advLaneSec.centerline.project(egoSpawnPt offset by -globalParameters.OPT_ADV_BEHIND_DIST @ 0),
    with heading egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with regionContainedIn advLaneSec,
    with heading egoSpawnPt.heading,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

require distance to intersection >= 100