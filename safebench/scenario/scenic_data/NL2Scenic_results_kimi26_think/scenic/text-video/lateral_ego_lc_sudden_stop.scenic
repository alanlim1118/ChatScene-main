"""Scenario Description:

The ego vehicle travels forward on a multi-lane urban road under clear skies, following a black Ford Mondeo sedan in the center lane while a motorized tricycle carrying a large box travels in the right lane and a red bus is visible in the distance on the left. As the ego vehicle proceeds, it appears to initiate a maneuver to change lanes to the left, positioning itself near a silver sedan that is traveling in the adjacent left lane. However, traffic in the left lane slows down abruptly, preventing the lane change from being completed. Simultaneously, the black lead vehicle's brake lights illuminate as it decelerates rapidly. Unable to clear the lane or stop in time due to the sudden slowing of traffic in both the target lane and the current lane, the ego vehicle collides with the rear bumper of the black Ford Mondeo, coming to a halt directly behind it while the silver sedan remains alongside in the left lane.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED * 0.6
param OPT_SILVER_SPEED = globalParameters.OPT_EGO_SPEED * 0.75
param OPT_TRIKE_SPEED = globalParameters.OPT_EGO_SPEED * 0.7
param OPT_BUS_SPEED = globalParameters.OPT_EGO_SPEED * 0.5

param OPT_LEAD_DIST = Range(15, 20)
param OPT_SILVER_DIST = Range(10, 14)
param OPT_TRIKE_DIST = Range(5, 10)
param OPT_BUS_DIST = Range(70, 90)

param OPT_LANE_CHANGE_DIST = Range(12, 16)
param OPT_BRAKE_TRIGGER_DIST = Range(18, 22)
param OPT_BLOCK_DIST = Range(3, 5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED) until (distance from self to ego < globalParameters.OPT_BRAKE_TRIGGER_DIST)
    while True:
        take SetBrakeAction(1)

behavior SilverBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_SILVER_SPEED) until (distance from self to ego < globalParameters.OPT_BRAKE_TRIGGER_DIST)
    while True:
        take SetBrakeAction(1)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to LeadVehicle < globalParameters.OPT_LANE_CHANGE_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to SilverSedan < globalParameters.OPT_BLOCK_DIST):
        take SetBrakeAction(1)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
LeadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
SilverSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_SILVER_DIST
BusSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_BUS_DIST

rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
TrikeSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_TRIKE_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (center lane)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Black Ford Mondeo sedan lead vehicle (center lane)
LeadVehicle = new Car at LeadSpawnPt,
    with regionContainedIn egoLaneSec,
    with color Color(0, 0, 0),
    with behavior LeadBehavior()

# Silver sedan in adjacent left lane
SilverSedan = new Car at SilverSpawnPt,
    with regionContainedIn leftLaneSec,
    with color Color(0.75, 0.75, 0.75),
    with behavior SilverBehavior()

# Motorized tricycle proxy in right lane
Trike = new Car at TrikeSpawnPt,
    with regionContainedIn rightLaneSec,
    with color Color(0.5, 0.35, 0.2),
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRIKE_SPEED)

# Red bus visible in the distance on the left
Bus = new Car at BusSpawnPt,
    with regionContainedIn leftLaneSec,
    with color Color(0.8, 0, 0),
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED)

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150