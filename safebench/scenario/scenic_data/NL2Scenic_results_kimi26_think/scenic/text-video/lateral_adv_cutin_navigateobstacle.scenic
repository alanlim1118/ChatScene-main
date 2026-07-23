"""Scenario Description:

The ego vehicle proceeds straight on a multi-lane city road under bright, hazy sunlight that creates significant glare on the windshield, following a large box truck in the same lane. A black sedan suddenly enters the frame from the right, cutting sharply into the ego vehicle's lane in an attempt to bypass a stationary black vehicle stopped in the rightmost lane or shoulder. This abrupt maneuver causes the black sedan to cross directly into the ego vehicle's path, resulting in a side-impact collision. The black sedan comes to a halt in the lane ahead of the ego vehicle, while the stationary vehicle it was avoiding remains visible to the right.

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
TRUCK_MODEL = "vehicle.carlamotors.carlacola"
SEDAN_MODEL = "vehicle.audi.a2"
STATIONARY_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_TRUCK_SPEED = Range(8, 12)
param OPT_SEDAN_SPEED = Range(14, 18)
param OPT_TRUCK_DIST = Range(15, 25)
param OPT_STATIONARY_DIST = Range(25, 40)
param OPT_LANE_CHANGE_TRIGGER_DIST = Range(10, 18)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to SedanAgent < 3):
        take SetBrakeAction(1)
        do WaitBehavior()

behavior TruckBehavior(truck_speed):
    do FollowLaneBehavior(target_speed=truck_speed)

behavior SedanCutInBehavior(sedan_speed, stationary_vehicle, target_lane_sec, trigger_dist):
    do FollowLaneBehavior(target_speed=sedan_speed) until (distance from self to stationary_vehicle < trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=sedan_speed)
    while True:
        take SetBrakeAction(1)

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

rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)

truckSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_TRUCK_DIST
stationarySpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_STATIONARY_DIST
sedanSpawnPt = new OrientedPoint at rightLanePt, with heading egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

StationaryVehicle = new Car at stationarySpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint STATIONARY_MODEL,
    with color (0, 0, 0),
    with behavior WaitBehavior()

SedanAgent = new Car at sedanSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint SEDAN_MODEL,
    with color (0, 0, 0),
    with behavior SedanCutInBehavior(
        globalParameters.OPT_SEDAN_SPEED,
        StationaryVehicle,
        egoLaneSec,
        globalParameters.OPT_LANE_CHANGE_TRIGGER_DIST
    )

TruckAgent = new Car at truckSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior TruckBehavior(globalParameters.OPT_TRUCK_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

require distance to intersection >= 80