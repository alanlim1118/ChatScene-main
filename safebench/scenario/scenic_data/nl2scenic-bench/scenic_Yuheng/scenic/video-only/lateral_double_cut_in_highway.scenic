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

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
SUV_MODEL = "vehicle.tesla.modely"
DISTRACTOR_MODEL = "vehicle.nissan.patrol"

# Speeds in m/s (104 km/h ≈ 28.9 m/s, 78 km/h ≈ 21.7 m/s, 62 km/h ≈ 17.2 m/s)
EGO_INITIAL_SPEED = 28.9
EGO_BRAKE_THRESHOLD_DIST = Range(35, 50)
SUV_CUTIN_TRIGGER_DIST = Range(40, 60)
SUV_FINAL_STOP_DIST = Range(8, 12)

BRAKE_FULL = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoHighwayBehavior(initial_speed, brake_threshold):
    try:
        do FollowLaneBehavior(target_speed=initial_speed)
    interrupt when (distance from self to CutInSUV < brake_threshold):
        take SetThrottleAction(0), SetBrakeAction(BRAKE_FULL)
        do WaitBehavior() for 10 seconds
        terminate

behavior CutInAndBrakeBehavior(cut_in_trigger_dist, target_lane):
    # Drive in right lane until trigger distance
    do FollowLaneBehavior(target_speed=globalParameters.EGO_INITIAL_SPEED) until (distance from self to ego < cut_in_trigger_dist)
    # Perform lane change to left (ego's lane)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=globalParameters.EGO_INITIAL_SPEED)
    # Immediately brake hard after cut-in
    take SetThrottleAction(0), SetBrakeAction(BRAKE_FULL)
    do WaitBehavior() for 10 seconds

behavior DistractorRecklessBehavior():
    # Simulate reckless maneuvering near highway split
    try:
        do FollowLaneBehavior(target_speed=Range(20, 30))
    interrupt when (distance from self to ego < 80):
        do LaneChangeBehavior(laneSectionToSwitch=Uniform(*self.lane.adjacentLanes), target_speed=Range(15, 25))
        do FollowLaneBehavior(target_speed=Range(10, 20))

#################################
# SPATIAL RELATIONS             #
#################################

# Find highway lanes with adjacent right lanes suitable for cut-in scenario
highwayLaneSecs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToRight is not None and sec._laneToRight.isForward:
            if sec.speedLimit >= 25:  # Highway-like speed limit
                highwayLaneSecs.append(sec)

require len(highwayLaneSecs) > 0

egoLaneSec = Uniform(*highwayLaneSecs)
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place SUV in right lane ahead of ego
rightProj = rightLaneSec.centerline.project(egoSpawnPt.position)
suvSpawnPt = new OrientedPoint following roadDirection from rightProj for Range(50, 80)

# Place distractor vehicles near a highway split area ahead
splitRegion = Region.fromPolyline([p for p in egoLaneSec.centerline.points])
distractorSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(100, 160)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime weather
param timeOfDay = 'night'
param weather = 'clear'

# Ego vehicle with HUD (simulated via blueprint; CARLA HUD is client-side)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoHighwayBehavior(EGO_INITIAL_SPEED, EGO_BRAKE_THRESHOLD_DIST),
    with name 'EgoWithHUD'

# Black SUV that cuts in and brakes
CutInSUV = new Car at suvSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint SUV_MODEL,
    with color (0, 0, 0),
    with behavior CutInAndBrakeBehavior(SUV_CUTIN_TRIGGER_DIST, egoLaneSec)

# Distractor vehicle simulating reckless behavior at highway split
DistractorCar = new Car following roadDirection from distractorSpawnPt for Range(-10, 10),
    with regionContainedIn egoLaneSec,
    with blueprint DISTRACTOR_MODEL,
    with behavior DistractorRecklessBehavior()

# Ensure sufficient distance from intersections for highway scenario
require distance to intersection >= 150

# Terminate when ego has come to a complete stop near the SUV
terminate when ego.speed < 0.5 and (distance from ego to CutInSUV) < SUV_FINAL_STOP_DIST + 5