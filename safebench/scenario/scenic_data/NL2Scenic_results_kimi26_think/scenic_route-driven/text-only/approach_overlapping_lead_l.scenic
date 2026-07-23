"""Scenario Description:

The ego car travels straight forward within its lane. It approaches a leading adversarial object that is overlapping its lane to the left.

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

param OPT_ADV_DIST = Range(15, 40)   # Distance ahead to place the adversarial object

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

# Adversarial object in the left lane, placed ahead of the ego on the lane boundary
advLaneSec = egoLaneSec._laneToLeft
advCenterBase = advLaneSec.centerline.project(egoSpawnPt.position)
advCenterAhead = new OrientedPoint following roadDirection from advCenterBase for globalParameters.OPT_ADV_DIST
advSpawnPos = advLaneSec.rightEdge.project(advCenterAhead.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

# Adversarial object overlapping the ego lane from the left
AdvAgent = new Car at advSpawnPos,
    with heading advCenterAhead.heading,
    with behavior StationaryBehavior()

require distance to intersection >= 50  # Ensure the ego vehicle is far from any intersection
