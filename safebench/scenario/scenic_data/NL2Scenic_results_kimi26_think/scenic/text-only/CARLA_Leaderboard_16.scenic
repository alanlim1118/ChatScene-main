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

param OPT_EGO_SPEED = Range(6, 10)
param OPT_HAZARD_SPEED = Range(1, 3)
param OPT_HAZARD_DIST = Range(25, 40)
param OPT_ONCOMING_DIST = Range(25, 40)
param OPT_ONCOMING_SPEED = Range(6, 10)
param OPT_BRAKE_DIST = Range(10, 15)

OPT_BRAKE_ACTION = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to Hazard) < globalParameters.OPT_BRAKE_DIST:
        take SetBrakeAction(OPT_BRAKE_ACTION)

behavior HazardBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_HAZARD_SPEED)

behavior OncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ONCOMING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

HazardSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_HAZARD_DIST

adjLaneSec = egoLaneSec._laneToLeft
OncomingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ONCOMING_DIST
OncomingSpawnPt = adjLaneSec.centerline.project(OncomingSpawnPt.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Slow-moving hazard blocking part of the lane ahead
Hazard = new Car at HazardSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior HazardBehavior()

# Oncoming traffic in the adjacent opposite lane
OncomingCar = new Car at OncomingSpawnPt,
    with heading egoSpawnPt.heading + 180 deg,
    with regionContainedIn adjLaneSec,
    with behavior OncomingBehavior()

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

require distance to intersection > 80
terminate when ego.speed < 0.5 and (distance to Hazard) < 20