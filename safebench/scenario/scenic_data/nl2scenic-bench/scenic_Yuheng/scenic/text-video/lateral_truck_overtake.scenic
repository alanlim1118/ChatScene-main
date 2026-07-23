"""Scenario Description:

The ego vehicle travels along a tree-lined two-lane road when a large red cargo truck initiates an overtaking maneuver from the left lane. As the truck passes, it encounters an oncoming motorcycle further down the road, forcing the driver to abruptly merge back into the right lane to avoid a head-on collision. This sudden return to the lane causes the truck to squeeze into the ego vehicle's path, resulting in a dangerous side-impact collision that forces the ego vehicle to decelerate significantly as the truck occupies the space directly ahead.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
TRUCK_MODEL = 'vehicle.carlamotors.carlacola'
MOTO_MODEL = 'vehicle.kawasaki.ninja'

param EGO_SPEED = VerifaiRange(8, 12)
param TRUCK_SPEED = VerifaiRange(10, 14)
param MOTO_SPEED = VerifaiRange(10, 15)

param EGO_BRAKE = VerifaiRange(0.7, 1.0)
param SAFETY_DIST = VerifaiRange(12, 20)
CRASH_DIST = 4
TERM_DIST = 120

# Distances for scenario setup
TRUCK_INIT_LATERAL_OFFSET = Range(-3.5, -4.5)  # Truck starts in left lane relative to ego
TRUCK_INIT_LONG_OFFSET = Range(-10, -5)        # Truck starts slightly behind/beside ego
MOTO_INIT_DIST = Range(60, 90)                 # Motorcycle is far ahead in opposite direction
TRUCK_OVERTAKE_TRIGGER = Range(15, 25)         # Distance ahead of ego where truck begins overtake
TRUCK_MERGE_BACK_DIST = Range(5, 12)           # Distance from motorcycle when truck merges back

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior TruckOvertakeBehavior(ego_ref, moto_ref, overtake_trigger, merge_back_dist):
    # Phase 1: Drive in left lane alongside ego
    do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) until (distance from self to ego_ref < overtake_trigger and relative position of ego_ref is behind self)
    
    # Phase 2: Continue overtaking until motorcycle is detected at merge_back_dist
    do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) until (distance from self to moto_ref < merge_back_dist)
    
    # Phase 3: Abruptly merge back to right lane (into ego's path)
    do LaneChangeBehavior(laneSectionToSwitch=RightLaneSection, target_speed=globalParameters.TRUCK_SPEED)
    
    # Phase 4: Continue in right lane ahead of ego after merge
    do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)

behavior OncomingMotoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.MOTO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight two-lane road section with adjacent lanes in same direction
# and an opposing lane for the motorcycle
candidateSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if (sec.isForward 
            and sec._laneToRight is not None 
            and sec._laneToRight.isForward
            and sec._laneToLeft is not None
            and sec._laneToLeft.isForward):
            candidateSections.append(sec)

require len(candidateSections) > 0
egoLaneSec = Uniform(*candidateSections)
leftLaneSec = egoLaneSec._laneToLeft
RightLaneSection = egoLaneSec._laneToRight

# Ego spawn point in right lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Truck spawns in left lane, slightly behind ego
truckBasePt = new OrientedPoint in leftLaneSec.centerline
truckSpawnPt = new OrientedPoint following roadDirection from truckBasePt for TRUCK_INIT_LONG_OFFSET

# Find opposing lane for motorcycle (same road, opposite direction)
opposingLanes = []
for lane in network.lanes:
    for sec in lane.sections:
        if not sec.isForward and sec.road is egoLaneSec.road:
            opposingLanes.append(sec)

require len(opposingLanes) > 0
motoLaneSec = Uniform(*opposingLanes)

# Motorcycle spawns far ahead on opposing lane
motoBasePt = new OrientedPoint in motoLaneSec.centerline
motoSpawnPt = new OrientedPoint following roadDirection from motoBasePt for MOTO_INIT_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn egoLaneSec,
    with behavior EgoBehavior()

truck = new Car at truckSpawnPt,
    with blueprint TRUCK_MODEL,
    with color "red",
    with regionContainedIn leftLaneSec,
    with behavior TruckOvertakeBehavior(ego, moto, TRUCK_OVERTAKE_TRIGGER, TRUCK_MERGE_BACK_DIST)

moto = new Car at motoSpawnPt,
    with blueprint MOTO_MODEL,
    with regionContainedIn motoLaneSec,
    with behavior OncomingMotoBehavior()

# Ensure minimum separation constraints
require distance from ego to truck >= 5
require distance from ego to moto >= 40

terminate when (distance from ego to egoSpawnPt) > TERM_DIST