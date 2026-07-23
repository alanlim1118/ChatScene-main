"""Scenario Description:

The ego vehicle drives forward on a wet, two-lane mountain road surrounded by dense vegetation under overcast conditions, initially traveling at roughly 82 km/h. As the vehicle rounds a gentle left curve marked by roadside chevron signs, an oncoming red truck appears in the opposite lane, followed closely by a black SUV. The black SUV attempts to overtake the truck by crossing the center line, directly invading the ego vehicle's lane and creating an imminent head-on collision threat. In response, the ego vehicle executes a sudden emergency brake, rapidly reducing speed as the black SUV passes extremely close to the front left side of the car. The ego vehicle continues to decelerate sharply, coming to a complete stop on the right side of the road immediately after the dangerous encounter.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
TRUCK_MODEL = "vehicle.carlamotors.firetruck"
SUV_MODEL = "vehicle.tesla.model3"

# Speeds in m/s (82 km/h ~ 22.78 m/s)
EGO_SPEED = 22.78
TRUCK_SPEED = 15.0
SUV_SPEED = 20.0

# Distances
TRUCK_DISTANCE_AHEAD = Range(60, 80)       # Truck initial distance ahead in opposite lane
SUV_FOLLOW_DISTANCE = Range(15, 25)        # SUV follows truck at this distance
EGO_BRAKE_TRIGGER_DIST = Range(35, 50)     # Distance to SUV when ego starts braking
SUV_OVERTAKE_TRIGGER_DIST = Range(40, 55)  # Distance to truck when SUV begins overtaking

BRAKE_ACTION = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoEmergencyBrakeBehavior(speed, brake_trigger_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (distance from self to SuvAgent < brake_trigger_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(BRAKE_ACTION)
        do WaitBehavior() for 10 seconds
        terminate

behavior TruckOncomingBehavior(speed):
    # Drive in opposite direction along its lane
    do FollowLaneBehavior(target_speed=speed)

behavior SuvOvertakeBehavior(speed, overtake_trigger_dist, truck_ref):
    # Follow lane behind truck until trigger distance, then change to oncoming (ego's) lane
    try:
        do FollowLaneBehavior(target_speed=speed) until (distance from self to truck_ref < overtake_trigger_dist)
        do LaneChangeBehavior(laneSectionToSwitch=left, target_speed=speed)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (self.speed < 0.5):
        take SetBrakeAction(BRAKE_ACTION)
        do WaitBehavior() for 5 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find two-lane road sections with opposing traffic
candidateSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToLeft is not None and not sec._laneToLeft.isForward:
            candidateSections.append(sec)

require len(candidateSections) > 0
egoLaneSec = Uniform(*candidateSections)
oppositeLaneSec = egoLaneSec._laneToLeft

# Ego spawn point on its lane centerline
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Truck spawn point in opposite lane, ahead of ego
truckRefPt = egoLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following roadDirection from truckRefPt for globalParameters.TRUCK_DISTANCE_AHEAD,
    with heading oppositeLaneSec.centerline.headingAt(truckRefPt.position)

# SUV spawn point behind truck in opposite lane
suvSpawnPt = new OrientedPoint following roadDirection from truckSpawnPt for -globalParameters.SUV_FOLLOW_DISTANCE,
    with heading truckSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather: wet, overcast
param weather = Weather(precipitation=0.8, cloudiness=1.0, wetness=1.0)

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoEmergencyBrakeBehavior(EGO_SPEED, EGO_BRAKE_TRIGGER_DIST)

# Oncoming red truck
TruckAgent = new Car at truckSpawnPt,
    with regionContainedIn oppositeLaneSec,
    with blueprint TRUCK_MODEL,
    with color "Red",
    with behavior TruckOncomingBehavior(TRUCK_SPEED)

# Black SUV that overtakes into ego's lane
SuvAgent = new Car at suvSpawnPt,
    with regionContainedIn oppositeLaneSec,
    with blueprint SUV_MODEL,
    with color "Black",
    with behavior SuvOvertakeBehavior(SUV_SPEED, SUV_OVERTAKE_TRIGGER_DIST, TruckAgent)

# Ensure we are far enough from intersections for the maneuver
require distance to intersection >= 100

# Terminate when ego has stopped after the encounter
terminate when ego.speed < 0.1 and (distance from ego to SuvAgent) < 30