"""Scenario Description:

The ego-vehicle encounters a slow moving hazard blocking part of the lane. The ego-vehicle must brake or maneuver to avoid it next to a lane of traffic moving in the opposite direction.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_HAZARD_SPEED = Range(1, 3)
param OPT_HAZARD_DIST = Range(40, 60)           # Distance ahead of ego where hazard is placed
param OPT_ONCOMING_DIST = Range(80, 120)        # Distance ahead of hazard where oncoming car starts
param OPT_ONCOMING_SPEED = Range(8, 12)
param OPT_BRAKE_THRESHOLD = Range(15, 25)       # Distance at which ego brakes if maneuver not possible
param OPT_MANEUVER_THRESHOLD = Range(30, 45)    # Distance at which ego attempts lane change

OPT_EGO_BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior HazardBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior OncomingBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior EgoBehavior(ego_speed, brake_threshold, maneuver_threshold, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to Hazard < maneuver_threshold)
        # Attempt to maneuver into opposite lane to bypass hazard
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to Hazard > maneuver_threshold + 10)
        # Return to original lane after passing hazard
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to Hazard < brake_threshold):
        take SetBrakeAction(brake_amount)
        do WaitBehavior() for 5 seconds
        terminate
    interrupt when (distance from self to OncomingCar < brake_threshold) and (self.laneSection != egoLaneSec):
        # Abort maneuver and brake if oncoming car is too close
        take SetBrakeAction(brake_amount)
        do WaitBehavior() for 5 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an adjacent left lane going in the opposite direction
laneSecsWithOppositeLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
            laneSecsWithOppositeLeft.append(laneSec)

require len(laneSecsWithOppositeLeft) > 0

egoLaneSec = Uniform(*laneSecsWithOppositeLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place hazard ahead of ego in same lane
hazardSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_HAZARD_DIST

# Place oncoming car ahead of hazard in the opposite lane
adjOppLaneSec = egoLaneSec._laneToLeft
oncomingRefPt = adjOppLaneSec.centerline.project(hazardSpawnPt.position)
OncomingSpawnPt = new OrientedPoint following roadDirection from oncomingRefPt for globalParameters.OPT_ONCOMING_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_THRESHOLD,
        globalParameters.OPT_MANEUVER_THRESHOLD,
        OPT_EGO_BRAKE_AMOUNT
    )

# Slow-moving hazard blocking part of the lane
Hazard = new Car at hazardSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior HazardBehavior(globalParameters.OPT_HAZARD_SPEED)

# Oncoming traffic in opposite lane
OncomingCar = new Car at OncomingSpawnPt,
    with heading egoSpawnPt.heading + 180 deg,
    with regionContainedIn adjOppLaneSec,
    with behavior OncomingBehavior(globalParameters.OPT_ONCOMING_SPEED)

require distance to intersection > 100