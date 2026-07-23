"""Scenario Description:

In an urban area during daylight with clear weather conditions, a traffic scenario unfolds on a non-junction road segment with a posted speed limit of 55 mph, depicted in a top-down schematic view. A vehicle traveling in the upper lane is shown initiating a lane change maneuver into the lower adjacent lane, indicated by curved arrows crossing the dashed center line. As the vehicle merges to the right, it closes in on a lead vehicle traveling ahead in the target lane, while another vehicle continues forward in the original upper lane, illustrating a typical lane-changing or passing situation.

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

# Speed limit 55 mph ≈ 24.6 m/s; use realistic CARLA speeds
param OPT_EGO_SPEED = Range(18, 22)
param OPT_LEAD_SPEED = Range(14, 17)        # Lead vehicle in target lane is slower
param OPT_FOLLOW_SPEED = Range(18, 22)      # Following vehicle in original lane maintains speed
param OPT_LEAD_DIST = Range(30, 45)         # Distance from ego spawn to lead vehicle along target lane
param OPT_FOLLOW_DIST = Range(-10, -5)      # Following vehicle starts slightly behind ego in original lane
param OPT_LANE_CHANGE_TRIGGER = Range(15, 25)  # Distance to lead vehicle that triggers lane change
param OPT_BRAKE_DIST = Range(6, 9)          # Safety braking distance

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to LeadVehicle < globalParameters.OPT_LANE_CHANGE_TRIGGER)
        do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right adjacent lane (for merging right)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

require len(laneSecsWithRightLane) > 0

egoLaneSec = Uniform(*laneSecsWithRightLane)
targetLaneSec = egoLaneSec._laneToRight

# Spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Project ego position onto target lane to get aligned reference
targetLaneRef = targetLaneSec.centerline.project(egoSpawnPt.position)
LeadSpawnPt = new OrientedPoint following roadDirection from targetLaneRef for globalParameters.OPT_LEAD_DIST

# Following vehicle in original lane, slightly behind ego
FollowSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_FOLLOW_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

LeadVehicle = new Car at LeadSpawnPt,
    with regionContainedIn targetLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)

FollowVehicle = new Car at FollowSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_FOLLOW_SPEED)

# Ensure we are on a non-junction road segment
require distance to intersection >= 100

# Terminate after sufficient travel distance
terminate when distance from ego to egoSpawnPt > 150