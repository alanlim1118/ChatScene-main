"""Scenario Description:

The ego car travels straight forward within the middle lane. Meanwhile, an adversarial object cuts ahead by turning from the left lane toward the right.

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

param OPT_EGO_SPEED = Range(8, 12)          # Speed for the ego vehicle
param OPT_ADV_SPEED = Range(5, 8)           # Speed for the adversarial vehicle
param OPT_ADV_START_DIST = Range(15, 30)    # Adversary starts this far ahead in the left lane
param OPT_ADV_CUT_DIST = Range(10, 20)      # Distance at which adversary initiates cut-in

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdversarialBehavior():
    # Drive in left lane until ego gets close, then cut right into middle lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until distance from self to ego < globalParameters.OPT_ADV_CUT_DIST
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec._laneToRight, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify middle lane sections that have both left and right adjacent lanes
laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and
            laneSec._laneToRight is not None and laneSec._laneToRight.isForward):
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneSec = egoLaneSec._laneToLeft
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup (in middle lane)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Adversarial agent in left lane, ahead of ego
AdvAgent = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with behavior AdversarialBehavior()

require distance to intersection >= 100  # Ensure the ego vehicle is far from the intersection
terminate when distance from ego to AdvAgent > 50