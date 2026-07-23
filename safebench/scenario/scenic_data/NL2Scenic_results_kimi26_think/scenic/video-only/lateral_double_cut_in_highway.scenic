"""Scenario Description:

Under nighttime conditions on a highway, the ego vehicle travels at 104 km/h with a heads-up display visible on the windshield showing navigation data. A black SUV suddenly cuts in from the right lane directly in front of the ego vehicle, its brake lights engaging instantly as it slows down. The ego vehicle initiates emergency braking, causing its speed to drop rapidly from 104 km/h to 78 km/h, then 62 km/h, and eventually to a complete stop of 0 km/h just behind the stationary SUV. This hazardous event is triggered by reckless maneuvers from passenger vehicles at a highway split, leading to a rear-end collision situation that forces the ego vehicle to brake abruptly to avoid a crash.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

# Nighttime setting
param time = 22.0

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
SUV_MODEL = "vehicle.audi.etron"

# Speeds in m/s
EGO_SPEED = 28.89          # 104 km/h
SUV_SPEED = 22.22          # 80 km/h

# Staged braking speed thresholds (m/s)
SPEED_78_KMH = 21.67
SPEED_62_KMH = 17.22

param OPT_CUT_IN_DIST = Range(30, 40)       # Initial distance of SUV ahead in right lane
param OPT_CUT_IN_TRIGGER = Range(20, 30)    # Distance at which SUV cuts in front of ego
param OPT_BRAKE_TRIGGER = Range(15, 20)     # Distance at which ego initiates emergency braking

OPT_EGO_BRAKE_STAGE1 = 0.5
OPT_EGO_BRAKE_STAGE2 = 0.7
OPT_EGO_BRAKE_STAGE3 = 1.0

OPT_SUV_BRAKE = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_BRAKE_TRIGGER):
        # Stage 1: Brake from 104 km/h down to 78 km/h
        while self.speed > SPEED_78_KMH:
            take SetBrakeAction(OPT_EGO_BRAKE_STAGE1)
        # Stage 2: Brake from 78 km/h down to 62 km/h
        while self.speed > SPEED_62_KMH:
            take SetBrakeAction(OPT_EGO_BRAKE_STAGE2)
        # Stage 3: Brake to a complete stop
        while self.speed > 0.5:
            take SetBrakeAction(OPT_EGO_BRAKE_STAGE3)
        do WaitBehavior()

behavior SUVCutInBehavior(ego_vehicle, target_lane, target_speed):
    # Drive in right lane until ego is close enough
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego_vehicle < globalParameters.OPT_CUT_IN_TRIGGER)
    # Cut in front of ego
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    # Brake instantly
    while True:
        take SetBrakeAction(OPT_SUV_BRAKE)

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

# SUV spawn point in right lane, ahead of ego
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
suvSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_CUT_IN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (left lane, highway) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

# --- Black SUV (right lane, cuts in and brakes) ---
suv = new Car at suvSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint SUV_MODEL,
    with color Color(0, 0, 0),
    with behavior SUVCutInBehavior(ego, egoLaneSec, SUV_SPEED)

# Place scenario near a highway split / interchange
require distance to intersection < 150
require distance to intersection > 50

# Terminate once ego has stopped close behind the SUV
terminate when ego.speed < 0.5 and (distance from ego to suv) < 20
terminate after 45 seconds