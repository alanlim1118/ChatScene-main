"""Scenario Description:

A blue ego vehicle traveling in the center lane of a multi-lane road attempts a lane change to the left but executes an aborted maneuver, curving its path toward the left lane before steering back into the center. This action occurs while two pink adversarial vehicles travel straight in the adjacent left lane, positioned as both a lead object ahead and a following object behind the blue car's intended merge point, effectively blocking the lane change.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(3, 6)
param OPT_LEAD_DIST = Range(10, 15)
param OPT_FOLLOW_DIST = Range(10, 15)
param OPT_BLOCK_DIST = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvLead < globalParameters.OPT_BLOCK_DIST or distance from self to AdvFollow < globalParameters.OPT_BLOCK_DIST):
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify forward center lane sections with both left and right neighbors
laneSecsCenter = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and
            laneSec._laneToRight is not None and laneSec._laneToRight.isForward):
            laneSecsCenter.append(laneSec)

egoLaneSec = Uniform(*laneSecsCenter)
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
adjLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)

# Adversarial lead vehicle: ahead in left lane
AdvLeadSpawn = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_LEAD_DIST

# Adversarial follow vehicle: behind in left lane
AdvFollowSpawn = new OrientedPoint following roadDirection from adjLanePt for -globalParameters.OPT_FOLLOW_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (blue) in center lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color [0, 0, 1],
    with behavior EgoBehavior()

# Adversarial lead vehicle (pink) in left lane, ahead of merge point
AdvLead = new Car at AdvLeadSpawn,
    with heading egoSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with color [1, 0.4, 0.7],
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

# Adversarial follow vehicle (pink) in left lane, behind merge point
AdvFollow = new Car at AdvFollowSpawn,
    with heading egoSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with color [1, 0.4, 0.7],
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150