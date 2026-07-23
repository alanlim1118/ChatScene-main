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

param OPT_EGO_SPEED = Range(5, 10)  # Speed of the ego vehicle in m/s
param OPT_ADV_DIST = Range(15, 40)   # Distance ahead to place the adversarial object

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify lane sections with a left lane (ego will be in the right lane)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

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
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Adversarial object overlapping the ego lane from the left
AdvAgent = new Car at advSpawnPos,
    with heading advCenterAhead.heading,
    with behavior StationaryBehavior()

require distance to intersection >= 50  # Ensure the ego vehicle is far from any intersection