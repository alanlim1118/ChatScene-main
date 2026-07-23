"""Scenario Description:

The ego-vehicle encounters a pedestrian emerging from behind a parked vehicle and advancing into the lane. The ego-vehicle must brake or maneuver to avoid it.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_PED_SPEED = Range(1.0, 2.0)
param OPT_PARKED_CAR_DISTANCE = Range(25, 40)      # Distance ahead of ego where parked car is placed
param OPT_PED_TRIGGER_DISTANCE = Range(12, 18)     # Distance from ego to pedestrian at which ped starts crossing
param OPT_BRAKE_TRIGGER_DISTANCE = Range(8, 12)    # Distance at which ego brakes if ped is in lane
param OPT_PED_OFFSET_BEHIND_CAR = Range(1, 3)      # How far behind/ahead of parked car the pedestrian spawns
param OPT_PED_LATERAL_OFFSET = Range(2, 4)         # Lateral offset from parked car toward sidewalk

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveAndBrake():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.OPT_BRAKE_TRIGGER_DISTANCE):
        take SetThrottleAction(0), SetBrakeAction(1)
        terminate

behavior PedestrianEmergeFromBehindCar(car_ref, trigger_dist, walk_speed):
    # Wait until ego is close enough before emerging
    while distance from self to ego > trigger_dist:
        wait
    # Walk into the lane (perpendicular to road direction, toward centerline)
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(walk_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight lane section for the scenario
straightLaneSecs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec._laneToLeft is not None or sec._laneToRight is not None:
            straightLaneSecs.append(sec)

egoLaneSec = Uniform(*straightLaneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Parked car position: ahead of ego along the lane
parkedCarPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_PARKED_CAR_DISTANCE

# Pedestrian spawn point: beside/behind the parked car on the right side (sidewalk side)
pedSpawnPt = new OrientedPoint right of parkedCarPt by globalParameters.OPT_PED_LATERAL_OFFSET,
    with heading parkedCarPt.heading + 90 deg   # Facing into the lane

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoDriveAndBrake()

# Parked vehicle blocking part of the lane / acting as occluder
ParkedCar = new Car at parkedCarPt,
    with heading parkedCarPt.heading,
    with regionContainedIn None

# Pedestrian emerging from behind the parked car
Pedestrian = new Pedestrian at pedSpawnPt,
    with heading parkedCarPt.heading + 90 deg,
    with regionContainedIn None,
    with behavior PedestrianEmergeFromBehindCar(
        ParkedCar,
        globalParameters.OPT_PED_TRIGGER_DISTANCE,
        globalParameters.OPT_PED_SPEED
    )

# Ensure sufficient distance for the scenario to play out
require distance from egoSpawnPt to parkedCarPt >= 20
require distance from parkedCarPt to pedSpawnPt <= 5

terminate after 30 seconds