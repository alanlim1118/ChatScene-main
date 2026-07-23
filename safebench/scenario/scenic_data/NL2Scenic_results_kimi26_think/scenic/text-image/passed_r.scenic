"""Scenario Description:

A top-down schematic illustration depicts a two-lane road divided by a dashed white line where a blue rectangular ego vehicle travels straight in the upper left lane. In the adjacent lower right lane, a pink rectangular adversarial vehicle accelerates forward, indicated by a longer arrow extending from its rear, demonstrating a scenario where an object is passing the ego vehicle on the right side.

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

param OPT_EGO_SPEED = Range(6, 10)           # Ego vehicle speed
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Range(1.3, 1.6)  # Adversarial vehicle speed (faster to pass)
param OPT_ADV_SPAWN_DIST = Range(15, 25)     # Distance behind ego in the right lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify lane sections that have a lane to the right (ego will be in the left lane)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial agent in the right lane, behind the ego vehicle
rightLaneSec = egoLaneSec._laneToRight
adjLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for -globalParameters.OPT_ADV_SPAWN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup (in left lane)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Adversarial vehicle setup (in right lane, behind ego, faster)
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior AdvBehavior()

require distance to intersection >= 100  # Ensure the ego vehicle is far from the intersection
terminate when (distance from ego to AdvAgent) > 50