"""Scenario Description:

The ego car travels straight forward within the middle lane. Ahead of it, the adversarial object moves across the lane, illustrating the scenario of a lead object entering from the right.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(5, 8)
param OPT_ADV_SPAWN_DIST = Range(25, 40)
param OPT_ADV_TRIGGER_DIST = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvBehavior():
    # Follow the right lane until the ego approaches, then cut into the middle lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego) < globalParameters.OPT_ADV_TRIGGER_DIST
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify forward lane sections that have both a left and a right neighbor (middle lane)
laneSecsMiddle = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToRight is not None:
            laneSecsMiddle.append(laneSec)

egoLaneSec = Uniform(*laneSecsMiddle)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial agent starts in the right lane, ahead of the ego
rightLaneSec = egoLaneSec._laneToRight
advBasePt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advBasePt for globalParameters.OPT_ADV_SPAWN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior AdvBehavior()

require distance to intersection >= 100