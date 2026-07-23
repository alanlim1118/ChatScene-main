"""Scenario Description:

The ego vehicle travels along a tree-lined two-lane road when a large red cargo truck initiates an overtaking maneuver from the left lane. As the truck passes, it encounters an oncoming motorcycle further down the road, forcing the driver to abruptly merge back into the right lane to avoid a head-on collision. This sudden return to the lane causes the truck to squeeze into the ego vehicle's path, resulting in a dangerous side-impact collision that forces the ego vehicle to decelerate significantly as the truck occupies the space directly ahead.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
TRUCK_MODEL = 'vehicle.carlamotors.carlacola'
MOTO_MODEL = 'vehicle.harley-davidson.low_rider'

param EGO_SPEED = Range(8, 12)
param TRUCK_SPEED = Range(10, 14)

param TRUCK_BEHIND = Range(8, 15)
param MOTO_AHEAD = Range(40, 60)
param MERGE_DIST = Range(15, 25)

SAFETY_DIST = 5
CRASH_DIST = 3
TERM_DIST = 150

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
        take SetBrakeAction(1.0)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior TruckBehavior(moto, rightLane, merge_dist):
    do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)
    until (distance from self to moto < merge_dist)
    
    do LaneChangeBehavior(laneSectionToSwitch=rightLane, target_speed=globalParameters.TRUCK_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)

behavior MotoBehavior():
    while True:
        take SetThrottleAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a forward right-lane section that has a forward lane to its left
twoLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is None:
            twoLaneSecs.append(laneSec)

egoLaneSec = Uniform(*twoLaneSecs)
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Truck reference point in left lane, aligned longitudinally with ego
truckRefPt = new OrientedPoint in leftLaneSec.centerline
require (distance from truckRefPt to egoSpawnPt) < 6
truckSpawnPt = new OrientedPoint at truckRefPt offset by -globalParameters.TRUCK_BEHIND @ 0

# Motorcycle reference point in left lane, aligned longitudinally with ego
motoRefPt = new OrientedPoint in leftLaneSec.centerline
require (distance from motoRefPt to egoSpawnPt) < 6
motoSpawnPt = new OrientedPoint at motoRefPt offset by globalParameters.MOTO_AHEAD @ 0, with heading (motoRefPt.heading + 3.14159)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

moto = new Car at motoSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint MOTO_MODEL,
    with behavior MotoBehavior()

truck = new Car at truckSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint TRUCK_MODEL,
    with color (1, 0, 0),
    with behavior TruckBehavior(moto, egoLaneSec, globalParameters.MERGE_DIST)

terminate when (distance to egoSpawnPt) > TERM_DIST