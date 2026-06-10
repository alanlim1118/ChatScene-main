"""Scenario Description:

The ego vehicle travels at a constant speed while a passenger car or motorcycle in the adjacent lane 
executes a cut-in maneuver with varying lateral velocities (simulated by varying speed during lane change) 
and longitudinal accelerations, forcing the ego vehicle to avoid a collision through braking.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Models
EGO_MODEL = "vehicle.lincoln.mkz_2017"
carModels = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.dodge.charger_police', 'vehicle.jeep.wrangler_rubicon', 'vehicle.mercedes.coupe', 'vehicle.mini.cooper_s', 'vehicle.ford.mustang', 'vehicle.nissan.micra', 'vehicle.nissan.patrol', 'vehicle.seat.leon', 'vehicle.tesla.model3', 'vehicle.toyota.prius', 'vehicle.volkswagen.t2']
motorcycleModels = ['vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja', 'vehicle.yamaha.yzf']

ADV_MODEL = Uniform(*(carModels + motorcycleModels))

# Speeds and Distances
param EGO_TARGET_SPEED = Range(10, 13)
param ADV_INIT_SPEED = Range(12, 15)
param ADV_CUT_IN_SPEED = Range(15, 18) # Higher speed for aggressive cut-in
param ADV_POST_CUT_SPEED = Range(8, 11) # Decelerate after cut-in to force ego braking

param BRAKE_THRESHOLD = Range(8, 12)
param INITIAL_GAP = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.BRAKE_THRESHOLD):
        # Emergency braking logic
        while self.speed > 0.1:
            take SetBrakeAction(1.0)
        do WaitBehavior()

behavior AdversaryCutInBehavior(target_lane_sec, init_speed, cut_speed, final_speed):
    # Phase 1: Drive in adjacent lane
    do FollowLaneBehavior(target_speed=init_speed) for Range(2, 4) seconds
    
    # Phase 2: Execute Cut-in (Lane Change)
    # Lateral velocity is influenced by the target_speed during LaneChangeBehavior
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=cut_speed)
    
    # Phase 3: Longitudinal Acceleration/Deceleration after cut-in
    do FollowLaneBehavior(target_speed=final_speed)

behavior WaitBehavior():
    while True:
        take SetBrakeAction(0.5)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that have a lane to the left (to allow a cut-in from the left lane to the right lane)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for ls in lane.sections:
        if ls.isForward and ls.laneToLeft is not None and ls.laneToLeft.isForward:
            laneSecsWithLeftLane.append(ls)

# Select a random valid lane section for the ego
egoLaneSec = Uniform(*laneSecsWithLeftLane)
advLaneSec = egoLaneSec.laneToLeft

# Spawn Points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Adversary starts ahead of the ego in the adjacent lane
advSpawnPt = new OrientedPoint at (egoSpawnPt offset by (Range(-2, 2) @ globalParameters.INITIAL_GAP)),
    facing egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego Vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_TARGET_SPEED)

# Adversary Vehicle (Car or Motorcycle)
adversary = new Vehicle at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryCutInBehavior(
        egoLaneSec, 
        globalParameters.ADV_INIT_SPEED, 
        globalParameters.ADV_CUT_IN_SPEED, 
        globalParameters.ADV_POST_CUT_SPEED
    )

# Environmental Constraints
require (distance to intersection) > 50
require (distance from adversary to intersection) > 50

# Termination
terminate when ego.speed < 0.5 and (distance to adversary) < 15 and adversary.speed > 1