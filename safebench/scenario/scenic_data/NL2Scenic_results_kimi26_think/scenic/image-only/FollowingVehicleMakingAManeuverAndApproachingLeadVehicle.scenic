"""Scenario Description:

In an urban area during daylight with clear weather conditions, a traffic scenario unfolds on a non-junction road segment with a posted speed limit of 55 mph, depicted in a top-down schematic view. A vehicle traveling in the upper lane is shown initiating a lane change maneuver into the lower adjacent lane, indicated by curved arrows crossing the dashed center line. As the vehicle merges to the right, it closes in on a lead vehicle traveling ahead in the target lane, while another vehicle continues forward in the original upper lane, illustrating a typical lane-changing or passing situation.

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

# Speeds in m/s (~55 mph ≈ 24.6 m/s)
param OPT_EGO_SPEED = Range(22, 25)
param OPT_LEAD_SPEED = Range(18, 21)
param OPT_OTHER_SPEED = Range(22, 25)
param OPT_LEAD_DIST = Range(35, 55)
param OPT_OTHER_DIST = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

# Ego initiates a lane change into the right (lower adjacent) lane
behavior EgoBehavior(target_lane, speed):
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a forward lane section that has a lane to the right
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
targetLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Lead vehicle is ahead in the target (lower adjacent) lane
targetProj = targetLaneSec.centerline.project(egoSpawnPt.position)
leadSpawnPt = new OrientedPoint following roadDirection from targetProj for globalParameters.OPT_LEAD_DIST
# Another vehicle continues forward in the original upper lane, placed behind the ego
otherSpawnPt = new OrientedPoint at egoSpawnPt offset by (-globalParameters.OPT_OTHER_DIST, 0)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetLaneSec, globalParameters.OPT_EGO_SPEED)

LeadVehicle = new Car at leadSpawnPt,
    with regionContainedIn targetLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)

OtherVehicle = new Car at otherSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_OTHER_SPEED)

# Ensure the scenario takes place on a non-junction road segment away from intersections
require distance to intersection >= 100

# Terminate after the ego has traveled a reasonable distance
terminate when distance from ego to egoSpawnPt > 150