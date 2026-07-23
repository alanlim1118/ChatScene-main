"""Scenario Description:

A blue ego vehicle traveling in the center lane of a multi-lane road attempts a lane change to the left but executes an aborted maneuver, curving its path toward the left lane before steering back into the center. This action occurs while two pink adversarial vehicles travel straight in the adjacent left lane, positioned as both a lead object ahead and a following object behind the blue car's intended merge point, effectively blocking the lane change.

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

param OPT_EGO_SPEED = Range(4, 6)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.95, 1.05)
param OPT_LEAD_DIST = Range(18, 25)       # Distance ahead of ego merge point for lead adversary
param OPT_FOLLOW_DIST = Range(15, 22)     # Distance behind ego merge point for following adversary
param OPT_ABORT_TRIGGER_DIST = Range(8, 12)  # Distance to lead adv at which ego aborts lane change
param OPT_BRAKE_DIST = Range(4, 6)        # Emergency braking distance threshold

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Start following the center lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to LeadAdv < globalParameters.OPT_ABORT_TRIGGER_DIST + 10)
    # Attempt lane change to the left
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to LeadAdv < globalParameters.OPT_ABORT_TRIGGER_DIST):
        # Abort: steer back to center lane
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    # Continue in center lane after abort
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a valid left lane (center lane with left neighbor)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            laneSec._laneToLeft.isForward and
            laneSec._laneToLeft._laneToLeft is not None  # Ensure left lane also has a left neighbor or is not edge
        ):
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point in center lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Project onto left lane centerline for adversary placement
leftLaneRefPt = leftLaneSec.centerline.project(egoSpawnPt.position)

# Lead adversary: ahead of the merge point in the left lane
leadAdvSpawnPt = new OrientedPoint following roadDirection from leftLaneRefPt for globalParameters.OPT_LEAD_DIST

# Following adversary: behind the merge point in the left lane
followAdvSpawnPt = new OrientedPoint following roadDirection from leftLaneRefPt for -globalParameters.OPT_FOLLOW_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue ego vehicle in center lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior()

# Pink lead adversarial vehicle in left lane (ahead)
LeadAdv = new Car at leadAdvSpawnPt,
    with heading leadAdvSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior AdvBehavior()

# Pink following adversarial vehicle in left lane (behind)
FollowAdv = new Car at followAdvSpawnPt,
    with heading followAdvSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior AdvBehavior()

require distance to intersection >= 100
terminate when (distance from ego to egoSpawnPt > 150)