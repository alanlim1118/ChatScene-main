"""Scenario Description:

Vehicle is passing another vehicle in a rural area, in daylight, under clear weather conditions, at a non-junction with a posted speed limit of 55 mph or more; and encroaches into another vehicle traveling in the opposite direction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural map with two-lane roads suitable for passing scenarios
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Speed parameters (m/s); 55 mph ≈ 24.6 m/s
MIN_SPEED_LIMIT = 24.6
EGO_TARGET_SPEED = Range(25, 30)
SLOW_CAR_SPEED = Range(15, 20)
ONCOMING_SPEED = Range(25, 30)

# Distance parameters
DISTANCE_TO_SLOW_CAR = Range(20, 30)
PASS_TRIGGER_DISTANCE = Range(15, 25)
ONCOMING_INITIAL_DISTANCE = Range(80, 120)
BRAKE_DIST = Range(8, 12)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoPassingBehavior():
    try:
        # Follow lane behind slow car until close enough to pass
        do FollowLaneBehavior(target_speed=EGO_TARGET_SPEED) until (distance from self to SlowCar < PASS_TRIGGER_DISTANCE)
        # Attempt to pass by changing to opposing lane
        do LaneChangeBehavior(laneSectionToSwitch=opposingLaneSec, target_speed=EGO_TARGET_SPEED)
        # Continue in opposing lane (encroachment)
        do FollowLaneBehavior(target_speed=EGO_TARGET_SPEED)
    interrupt when withinDistanceToObjsInLane(self, BRAKE_DIST):
        take SetBrakeAction(1.0)

behavior OncomingBehavior():
    while True:
        do FollowLaneBehavior(target_speed=ONCOMING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find road sections that are not part of any intersection and have an opposing lane
nonJunctionSectionsWithOpposing = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
            # Check that this section is not within any intersection
            inIntersection = False
            for intersec in network.intersections:
                if laneSec in intersec.lanes or laneSec._laneToLeft in intersec.lanes:
                    inIntersection = True
                    break
            if not inIntersection:
                nonJunctionSectionsWithOpposing.append(laneSec)

require len(nonJunctionSectionsWithOpposing) > 0

egoLaneSec = Uniform(*nonJunctionSectionsWithOpposing)
opposingLaneSec = egoLaneSec._laneToLeft

# Spawn points along the ego lane centerline
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
slowCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for DISTANCE_TO_SLOW_CAR

# Oncoming car spawns ahead in the opposing lane, traveling toward ego
oncomingRefPt = opposingLaneSec.centerline.project(slowCarSpawnPt.position)
oncomingSpawnPt = new OrientedPoint following roadDirection from oncomingRefPt for ONCOMING_INITIAL_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set environmental conditions: daylight, clear weather
param timeOfDay = 12
param weather = 'Clear'

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoPassingBehavior()

SlowCar = new Car at slowCarSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=SLOW_CAR_SPEED)

OncomingCar = new Car at oncomingSpawnPt,
    with regionContainedIn opposingLaneSec,
    with behavior OncomingBehavior()

# Ensure we are far from any intersection (non-junction requirement)
require distance to intersection >= 100

# Terminate after sufficient distance has been traveled
terminate when distance from ego to egoSpawnPt > 200