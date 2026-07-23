"""Scenario Description:

The ego-vehicle encounters an oncoming vehicle invading its lane on a bend due to an obstacle. It must brake or maneuver to the side of the road to navigate past the oncoming traffic.

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
ONCOMING_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(6, 9)
param OPT_ONCOMING_SPEED = Range(6, 9)
param OPT_BRAKE_DISTANCE = Range(15, 25)       # Distance at which ego reacts to oncoming vehicle
param OPT_SPAWN_DISTANCE = Range(40, 60)       # Initial distance between ego and oncoming vehicle along road
param OPT_OBSTACLE_OFFSET = Range(-1.5, -0.5)  # Lateral offset of obstacle in oncoming lane (pushes car into ego lane)

EGO_BRAKE_AMOUNT = 1.0
EGO_STEER_AMOUNT = 0.6                         # Moderate steer to side of road

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoAvoidanceBehavior(speed, brake_dist, brake_amount, steer_amount):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (distance from self to OncomingCar < brake_dist):
        take SetBrakeAction(brake_amount)
        take SetSteerAction(-steer_amount)      # Steer right toward road edge
        do WaitBehavior() for 3 seconds
        terminate

behavior OncomingInvadeBehavior(speed, obstacle):
    """Oncoming vehicle drives forward but is forced toward ego's lane by obstacle."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (distance from self to obstacle < 10):
        # Simulate swerving around obstacle into opposing lane
        take SetSteerAction(0.5)                # Steer left into ego's lane
        do FollowLaneBehavior(target_speed=speed * 0.7) for 4 seconds
        take SetSteerAction(0.0)
        do WaitBehavior() for 2 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an adjacent opposing lane (for bends with two-way traffic)
bendLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToLeft is not None and not sec._laneToLeft.isForward:
            bendLaneSections.append(sec)

require len(bendLaneSections) > 0

egoLaneSec = Uniform(*bendLaneSections)
oncomingLaneSec = egoLaneSec._laneToLeft

# Spawn ego on its lane centerline
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Spawn oncoming vehicle ahead on the opposing lane
oncomingBasePt = oncomingLaneSec.centerline.project(
    egoSpawnPt.position + Vector(0, globalParameters.OPT_SPAWN_DISTANCE).rotatedBy(egoSpawnPt.heading)
)
oncomingSpawnPt = new OrientedPoint at oncomingBasePt.position,
    with heading oncomingBasePt.heading

# Place obstacle in oncoming lane to force invasion
obstaclePos = oncomingLaneSec.centerline.pointAlongBy(
    distance=globalParameters.OPT_SPAWN_DISTANCE * 0.6,
    start=oncomingLaneSec.centerline.start
)
obstacle = new Trash at obstaclePos offset by globalParameters.OPT_OBSTACLE_OFFSET @ 0,
    with regionContainedIn oncomingLaneSec

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoAvoidanceBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_DISTANCE,
        EGO_BRAKE_AMOUNT,
        EGO_STEER_AMOUNT
    )

OncomingCar = new Car at oncomingSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with blueprint ONCOMING_MODEL,
    with behavior OncomingInvadeBehavior(globalParameters.OPT_ONCOMING_SPEED, obstacle)

require (distance from egoSpawnPt to intersection) >= 80
require curvatureAt(egoSpawnPt) > 0.001         # Ensure we are actually on a bend

terminate when ego.speed < 0.5 and (distance from ego to OncomingCar) < 20