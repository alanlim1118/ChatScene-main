"""Scenario Description:

This top-down schematic depicts a multi-lane roadway with traffic flowing from left to right, featuring a blue vehicle in the upper lane traveling straight ahead and a pink vehicle in the lower lane. The pink vehicle is shown performing a lane change maneuver, indicated by a curved arrow guiding it from the bottom lane into the central lane. The scenario is annotated with the text "Object entering to the right side of ego," which identifies the pink vehicle as the object of interest merging into the flow from the right-hand side relative to the ego vehicle's position.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
BLUE_CAR_MODEL = 'vehicle.tesla.model3'
PINK_CAR_MODEL = 'vehicle.mini.cooper_s'

param EGO_SPEED = Range(8, 12)
param PINK_SPEED = Range(8, 12)
param BLUE_SPEED = Range(8, 12)

param LANE_CHANGE_DIST = Range(30, 60)
param BRAKE_DIST = Range(8, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance from self to pinkCar < globalParameters.BRAKE_DIST):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
    terminate

behavior PinkCarBehavior(targetLaneSec):
    try:
        do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.PINK_SPEED)
    interrupt when (distance from self to ego < globalParameters.BRAKE_DIST):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left and right forward lane (3+ lane road)
laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            laneSecsWithLeftAndRight.append(laneSec)

require len(laneSecsWithLeftAndRight) > 0

# Ego is in the center lane
egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

# Spawn point for ego on center lane centerline
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

# Blue car spawn point on the left (upper) lane, ahead of ego
blueLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
blueSpawnPt = follow roadDirection from blueLanePt for Range(10, 30)

# Pink car spawn point on the right (lower) lane, slightly behind or alongside ego
pinkLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
pinkSpawnPt = follow roadDirection from pinkLanePt for Range(-10, 5)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in center lane (blue in schematic context, but ego is the reference)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    facing roadDirection,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Blue vehicle in upper (left) lane traveling straight
blueCar = new Car at blueSpawnPt,
    with regionContainedIn leftLaneSec,
    facing roadDirection,
    with blueprint BLUE_CAR_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.BLUE_SPEED)

# Pink vehicle in lower (right) lane performing lane change into center lane
pinkCar = new Car at pinkSpawnPt,
    with regionContainedIn rightLaneSec,
    facing roadDirection,
    with blueprint PINK_CAR_MODEL,
    with behavior PinkCarBehavior(targetLaneSec=egoLaneSec)

# Ensure valid spatial configuration
require distance from ego to pinkCar >= 5
require distance from ego to blueCar >= 5
require laneSecsWithLeftAndRight is not None