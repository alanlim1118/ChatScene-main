"""Scenario Description:

On a bright, sunny day along a wide multi-lane city road flanked by tall apartment buildings, the ego vehicle proceeds forward in the middle lane. A green and yellow taxi, initially traveling in the lane to the right, suddenly executes a sharp and aggressive lane change, cutting across the lane markings directly into the ego vehicle's path. This abrupt maneuver places the taxi immediately in front of the ego vehicle, necessitating heavy braking to prevent a rear-end collision. The taxi stabilizes in the lane ahead, and the ego vehicle maintains a safe following distance as traffic flow resumes normalcy.

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

param OPT_EGO_SPEED = Range(10, 14)
param OPT_TAXI_SPEED = Range(8, 12)
param OPT_CUTIN_DIST = Range(25, 35)       # Initial distance of taxi ahead in right lane
param OPT_CUTIN_TRIGGER = Range(10, 15)    # Distance at which taxi cuts in (ego behind)
param OPT_BRAKE_DIST = Range(6, 10)        # Distance at which ego brakes

OPT_FULL_BRAKE = 1

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to taxi) < brake_dist:
        take SetBrakeAction(OPT_FULL_BRAKE)
        do WaitBehavior() for 3 seconds
        do FollowLaneBehavior(target_speed=ego_speed)

behavior TaxiBehavior(taxi_speed, target_lane, trigger_dist):
    do FollowLaneBehavior(target_speed=taxi_speed) until (distance from ego to self) < trigger_dist
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=taxi_speed)
    do FollowLaneBehavior(target_speed=taxi_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find middle lane sections that have both left and right adjacent forward lanes
laneSecsMiddle = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToRight is not None and laneSec._laneToRight.isForward and
            laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward):
            laneSecsMiddle.append(laneSec)

egoLaneSec = Uniform(*laneSecsMiddle)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
rightLaneSec = egoLaneSec._laneToRight

rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
taxiSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_CUTIN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (middle lane, forward) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_DIST
    )

# --- Taxi (right lane, ahead of ego, cuts in aggressively) ---
taxi = new Car at taxiSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior TaxiBehavior(
        globalParameters.OPT_TAXI_SPEED,
        egoLaneSec,
        globalParameters.OPT_CUTIN_TRIGGER
    )

require (distance to intersection) >= 80