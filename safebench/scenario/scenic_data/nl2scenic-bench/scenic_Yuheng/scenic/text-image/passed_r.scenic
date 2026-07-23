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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(3, 5)          # Ego maintains moderate constant speed
param OPT_ADV_SPEED = Range(8, 12)         # Adversarial vehicle accelerates to pass
param OPT_ADV_START_DIST = Range(-15, -5)  # Adversary starts behind ego in right lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    """Ego vehicle drives straight in the left lane at constant speed."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvPassingBehavior():
    """Adversarial vehicle accelerates forward in the right lane to pass the ego."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right neighbor (ego in left, adversary in right)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial vehicle in the right lane, initially behind the ego
rightLaneSec = egoLaneSec._laneToRight
adjLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in the left lane (blue)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),       # Blue color for ego
    with behavior EgoBehavior()

# Adversarial vehicle in the right lane (pink), starting behind ego
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),   # Pink color for adversarial vehicle
    with behavior AdvPassingBehavior()

# Ensure we are far from intersections for clean straight-road passing
require distance to intersection >= 100

# Terminate once the adversarial vehicle has passed well ahead of ego
terminate when (distance from ego to AdvAgent) > 60 and AdvAgent.speed > globalParameters.OPT_EGO_SPEED