"""Scenario Description:

The ego vehicle travels along a multi-lane highway in clear daylight, positioned in the left lane with tall residential buildings visible to the left and a large tanker truck occupying the lane to the right. Directly ahead, a white SUV proceeds in the same lane until it suddenly locks its brakes, likely in response to a congested bottleneck or slowing traffic ahead. This abrupt deceleration forces the ego vehicle into an emergency braking scenario, but it is unable to stop in time and rear-ends the white SUV. The collision brings the ego vehicle to a halt directly behind the white car, while other vehicles, such as a silver van and a black SUV, continue to pass in the adjacent right lane.

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
SUV_MODEL = "vehicle.audi.etron"
TANKER_MODEL = "vehicle.carlamotors.carlacola"
VAN_MODEL = "vehicle.mercedes.sprinter"
BLACK_SUV_MODEL = "vehicle.nissan.patrol"

param OPT_EGO_SPEED = Range(13, 16)
param OPT_SUV_SPEED = Range(8, 10)
param OPT_SUV_DIST = Range(15, 20)
param OPT_BRAKE_TRIGGER = Range(5, 8)
param OPT_SUV_BRAKE_TIME = Range(2, 4)
param OPT_TANKER_OFFSET = Range(-5, 5)
param OPT_VAN_OFFSET = Range(-40, -25)
param OPT_BLACK_SUV_OFFSET = Range(-55, -40)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(ego_speed, brake_trigger):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to SUV) < brake_trigger:
        while True:
            take SetBrakeAction(1.0)

behavior SUVBehavior(speed, brake_time):
    do FollowLaneBehavior(target_speed=speed) for brake_time seconds
    while True:
        take SetBrakeAction(1.0)

behavior PassBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
rightLaneSec = egoLaneSec._laneToRight

# White SUV ahead in the same lane
SUVSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_SUV_DIST

# Tanker truck in the right lane, roughly alongside the ego
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
TankerSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_TANKER_OFFSET

# Passing vehicles in the right lane, behind the ego
VanSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_VAN_OFFSET
BlackSUVSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_BLACK_SUV_OFFSET

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (left lane) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_TRIGGER
    )

# --- White SUV ahead in left lane ---
SUV = new Car at SUVSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint SUV_MODEL,
    with behavior SUVBehavior(
        globalParameters.OPT_SUV_SPEED,
        globalParameters.OPT_SUV_BRAKE_TIME
    )

# --- Tanker truck in right lane ---
tanker = new Car at TankerSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint TANKER_MODEL,
    with behavior PassBehavior(globalParameters.OPT_EGO_SPEED - 2)

# --- Silver van in right lane ---
van = new Car at VanSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint VAN_MODEL,
    with behavior PassBehavior(globalParameters.OPT_EGO_SPEED)

# --- Black SUV in right lane ---
black_suv = new Car at BlackSUVSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint BLACK_SUV_MODEL,
    with behavior PassBehavior(globalParameters.OPT_EGO_SPEED)

require (distance to intersection) >= 100