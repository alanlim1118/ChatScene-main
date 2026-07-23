"""Scenario Description:

The ego car steers toward the right lane before veering back into its original path to perform an aborted lane change right. The adversarial car proceeds straight ahead in the adjacent right lane, acting as the lead object in this scenario.

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

param OPT_EGO_SPEED = Range(7, 11)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_ADV_SPAWN_DIST = Range(30, 50)
param OPT_START_DIST = Range(20, 30)
param OPT_ABORT_DIST = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Drive in the original lane until close enough to the adversary
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to AdvAgent) < globalParameters.OPT_START_DIST
    
    # Steer toward the right lane
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to AdvAgent) < globalParameters.OPT_ABORT_DIST
    
    # Veer back into the original lane to abort the maneuver
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    
    # Continue straight in the original lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify lane sections with a right lane (ego will be in the left lane)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rightLaneSec = egoLaneSec._laneToRight

# Spawn adversary ahead in the adjacent right lane
advSpawnBase = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_SPAWN_DIST
advSpawnPt = rightLaneSec.centerline.project(advSpawnBase.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Adversarial car setup in the right lane
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)