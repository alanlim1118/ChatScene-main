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
LEAD_MODEL = "vehicle.ford.montero"       # Black Ford Mondeo (using closest available)
TRICYCLE_MODEL = "vehicle.diamondback.century"  # Motorized tricycle proxy
BUS_MODEL = "vehicle.carlamotors.carlacola"     # Red bus proxy
SILVER_SEDAN_MODEL = "vehicle.tesla.model3"     # Silver sedan proxy

param OPT_EGO_SPEED = Range(6, 9)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.85, 0.95)
param OPT_SILVER_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.0, 1.1)
param OPT_TRICYCLE_SPEED = Range(2, 4)
param OPT_BUS_SPEED = Range(8, 12)

param OPT_LEAD_DIST = Range(18, 28)           # Distance to lead vehicle ahead
param OPT_SILVER_OFFSET = Range(-5, 5)        # Longitudinal offset of silver sedan relative to ego
param OPT_TRICYCLE_OFFSET = Range(5, 15)      # Tricycle ahead in right lane
param OPT_BUS_DIST = Range(60, 90)            # Bus far ahead in left lane

param OPT_LANE_CHANGE_TRIGGER_DIST = Range(12, 18)  # When ego starts lane change
param OPT_BRAKE_TRIGGER_DIST = Range(6, 10)         # When lead vehicle brakes hard
param OPT_SILVER_BRAKE_DIST = Range(8, 14)          # When silver sedan slows abruptly
param OPT_COLLISION_DIST = 3.0                       # Rear-end collision threshold

OPT_LEAD_BRAKE_AMOUNT = 1.0
OPT_SILVER_BRAKE_AMOUNT = 0.8
OPT_EGO_BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) \
            until (distance from self to LeadVehicle < globalParameters.OPT_LANE_CHANGE_TRIGGER_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to LeadVehicle < globalParameters.OPT_COLLISION_DIST):
        take SetBrakeAction(OPT_EGO_BRAKE_AMOUNT)
        do WaitBehavior() for 5 seconds
        terminate

behavior LeadVehicleBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED) \
        until (distance from self to ego < globalParameters.OPT_BRAKE_TRIGGER_DIST)
    while True:
        take SetBrakeAction(OPT_LEAD_BRAKE_AMOUNT)

behavior SilverSedanBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_SILVER_SPEED) \
        until (distance from self to ego < globalParameters.OPT_SILVER_BRAKE_DIST)
    while True:
        take SetBrakeAction(OPT_SILVER_BRAKE_AMOUNT)

behavior TricycleBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRICYCLE_SPEED)

behavior BusBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both left and right adjacent forward lanes (center lane of 3+)
laneSecsWithBothNeighbors = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            laneSecsWithBothNeighbors.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithBothNeighbors)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Lead vehicle spawn point (ahead in center lane)
LeadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

# Silver sedan spawn point (in left lane, roughly alongside ego)
leftLaneProjPt = leftLaneSec.centerline.project(egoSpawnPt.position)
SilverSpawnPt = new OrientedPoint following roadDirection from leftLaneProjPt for globalParameters.OPT_SILVER_OFFSET

# Tricycle spawn point (in right lane, ahead of ego)
rightLaneProjPt = rightLaneSec.centerline.project(egoSpawnPt.position)
TricycleSpawnPt = new OrientedPoint following roadDirection from rightLaneProjPt for globalParameters.OPT_TRICYCLE_OFFSET

# Bus spawn point (far ahead in left lane)
BusSpawnPt = new OrientedPoint following roadDirection from leftLaneProjPt for globalParameters.OPT_BUS_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in center lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Black Ford Mondeo lead vehicle in center lane
LeadVehicle = new Car at LeadSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint LEAD_MODEL,
    with color "black",
    with behavior LeadVehicleBehavior()

# Silver sedan in left lane
SilverSedan = new Car at SilverSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint SILVER_SEDAN_MODEL,
    with color "silver",
    with behavior SilverSedanBehavior()

# Motorized tricycle in right lane
Tricycle = new Car at TricycleSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint TRICYCLE_MODEL,
    with behavior TricycleBehavior()

# Red bus far ahead in left lane
Bus = new Car at BusSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint BUS_MODEL,
    with color "red",
    with behavior BusBehavior()

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150