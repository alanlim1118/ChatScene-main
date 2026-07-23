"""Scenario Description:

The ego car drives straight forward within its lane. Meanwhile, an adversarial motorcycle passes it from the adjacent left lane.

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
ADV_MODEL = "vehicle.yamaha.yzf"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Range(15, 20) / 10
param OPT_ADV_START_DIST = Range(10, 20) * -1

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversarialBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

adjLaneSec = egoLaneSec._laneToLeft
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following adjLaneSec.orientation from adjLanePt for globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

AdvAgent = new Motorcycle at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with blueprint ADV_MODEL,
    with behavior AdversarialBehavior()

require distance to intersection >= 50
