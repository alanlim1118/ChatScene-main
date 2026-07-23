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

param OPT_ADV_SPEED = Range(4, 7)
param OPT_ADV_SPAWN_DIST = Range(30, 50)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

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
    with blueprint EGO_MODEL

# Adversarial car setup in the right lane
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
