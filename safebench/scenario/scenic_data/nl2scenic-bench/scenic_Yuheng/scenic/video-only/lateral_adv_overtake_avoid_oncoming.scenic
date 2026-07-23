"""Scenario Description:

The ego vehicle is driving forward on a wet, two-lane rural road during a rainstorm, with windshield wipers active to clear the view. Ahead, a silver sedan travels in the same lane, while a white sedan attempts to overtake it by moving into the opposing lane. As the white sedan accelerates past, it encounters oncoming traffic, including a red car and a black SUV, forcing it to abort the overtake. To avoid a catastrophic head-on collision with the oncoming vehicles, the white sedan aggressively cuts back into the ego vehicle's lane, resulting in a side-impact collision as it squeezes back into the lane ahead of the ego vehicle.

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
SILVER_MODEL = "vehicle.tesla.model3"
WHITE_MODEL = "vehicle.audi.a2"
RED_MODEL = "vehicle.ford.mustang"
BLACK_SUV_MODEL = "vehicle.nissan.patrol"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SILVER_SPEED = globalParameters.OPT_EGO_SPEED - Uniform(2, 4)
param OPT_WHITE_OVERTAKE_SPEED = globalParameters.OPT_EGO_SPEED + Uniform(3, 6)
param OPT_ONCOMING_SPEED = Range(10, 15)

param OPT_SILVER_DIST = Range(25, 35)
param OPT_WHITE_START_DIST = Range(15, 20)
param OPT_ONCOMING_RED_DIST = Range(80, 100)
param OPT_ONCOMING_SUV_DIST = Range(110, 130)

param OPT_ABORT_TRIGGER_DIST = Range(30, 45)
param OPT_CUTBACK_LATERAL_OFFSET = Range(-0.5, 0.5)

OPT_WETNESS = 1.0
OPT_PRECIPITATION = 0.8

#################################
# AGENT BEHAVIORS               #
#################################

behavior SilverBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_SILVER_SPEED)

behavior WhiteOvertakeBehavior():
    # Start behind silver, accelerate and move to opposite lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_OVERTAKE_SPEED) until (distance from self to SilverCar < 10)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.OPT_WHITE_OVERTAKE_SPEED)
    # Continue overtaking until oncoming traffic is detected
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_OVERTAKE_SPEED) until (distance from self to RedOncoming < globalParameters.OPT_ABORT_TRIGGER_DIST)
    interrupt when (distance from self to RedOncoming < globalParameters.OPT_ABORT_TRIGGER_DIST):
        # Abort overtake: aggressively cut back into ego's lane
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_WHITE_OVERTAKE_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_OVERTAKE_SPEED)

behavior OncomingBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to WhiteCar < 8) and (WhiteCar.laneSection == self.laneSection):
        take SetBrakeAction(1)
        take SetThrottleAction(0)

#################################
# WEATHER SETUP                 #
#################################

weather = new Weather(
    precipitation=OPT_PRECIPITATION,
    cloudiness=0.9,
    windIntensity=0.6,
    fogDensity=0.3,
    wetness=OPT_WETNESS
)

#################################
# SPATIAL RELATIONS             #
#################################

# Find two-lane road sections with an opposing left lane
laneSecsWithOpposingLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            not laneSec._laneToLeft.isForward
        ):
            laneSecsWithOpposingLeft.append(laneSec)

require len(laneSecsWithOpposingLeft) > 0
egoLaneSec = Uniform(*laneSecsWithOpposingLeft)
opposingLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Silver car ahead of ego in same lane
SilverSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_SILVER_DIST

# White car starts between ego and silver, slightly behind silver
WhiteStartOffset = globalParameters.OPT_SILVER_DIST - globalParameters.OPT_WHITE_START_DIST
WhiteSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for WhiteStartOffset

# Oncoming traffic spawn points in opposing lane
RedOncomingBasePt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ONCOMING_RED_DIST
RedOncomingSpawnPt = opposingLaneSec.centerline.project(RedOncomingBasePt.position)

SUVOncomingBasePt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ONCOMING_SUV_DIST
SUVOncomingSpawnPt = opposingLaneSec.centerline.project(SUVOncomingBasePt.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(),
    with wipers True

# Silver sedan ahead in ego's lane
SilverCar = new Car at SilverSpawnPt,
    with heading SilverSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint SILVER_MODEL,
    with behavior SilverBehavior()

# White sedan that will attempt overtake then cut back
WhiteCar = new Car at WhiteSpawnPt,
    with heading WhiteSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint WHITE_MODEL,
    with behavior WhiteOvertakeBehavior()

# Oncoming red car in opposing lane
RedOncoming = new Car at RedOncomingSpawnPt,
    with heading RedOncomingSpawnPt.heading + 180 deg,
    with regionContainedIn opposingLaneSec,
    with blueprint RED_MODEL,
    with behavior OncomingBehavior(globalParameters.OPT_ONCOMING_SPEED)

# Oncoming black SUV further back in opposing lane
BlackSUV = new Car at SUVOncomingSpawnPt,
    with heading SUVOncomingSpawnPt.heading + 180 deg,
    with regionContainedIn opposingLaneSec,
    with blueprint BLACK_SUV_MODEL,
    with behavior OncomingBehavior(globalParameters.OPT_ONCOMING_SPEED)

require distance to intersection >= 150
terminate when (collision between ego and WhiteCar) or (distance from ego to WhiteCar > 100)