"""Scenario Description:

An autonomous driving system (ADS) equipped vehicle, depicted in green, travels in the left lane of a straight, multi-lane urban road and initiates a lane change maneuver to the right lane to prepare for a necessary turn. The target lane is occupied by two other vehicles, one positioned ahead and another behind, creating a specific gap between them. A vertical double-headed arrow labeled "Desired Merge Location" highlights this gap as the intended destination for the merging vehicle. The diagram illustrates a test scenario where the ADS vehicle must safely navigate into the space between the leading and trailing vehicles in the adjacent lane.

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

param OPT_EGO_SPEED = Range(7, 9)
param OPT_LEADING_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.6, 0.75)
param OPT_TRAILING_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.75, 0.9)
param OPT_MERGE_DISTANCE = Range(50, 70)
param OPT_GAP_FRONT = Range(20, 30)
param OPT_GAP_REAR = Range(20, 30)
param OPT_LANE_CHANGE_TRIGGER = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to mergePt < globalParameters.OPT_LANE_CHANGE_TRIGGER)
    do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToRight is not None and
            laneSec._laneToRight.isForward
        ):
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
rightLanePt = new OrientedPoint at rightLaneSec.centerline.project(egoSpawnPt.position)

mergePt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_MERGE_DISTANCE
LeadingSpawnPt = new OrientedPoint following roadDirection from rightLanePt for (globalParameters.OPT_MERGE_DISTANCE + globalParameters.OPT_GAP_FRONT)
TrailingSpawnPt = new OrientedPoint following roadDirection from rightLanePt for (globalParameters.OPT_MERGE_DISTANCE - globalParameters.OPT_GAP_REAR)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

LeadingAgent = new Car at LeadingSpawnPt,
    with heading LeadingSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

TrailingAgent = new Car at TrailingSpawnPt,
    with heading TrailingSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRAILING_SPEED)

require distance to intersection >= 100
terminate when (distance from ego to egoSpawnPt > 150)