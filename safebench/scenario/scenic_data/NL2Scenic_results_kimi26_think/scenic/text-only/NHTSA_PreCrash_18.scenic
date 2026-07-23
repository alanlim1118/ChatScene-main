"""Scenario Description:

Vehicle is changing lanes or passing in an urban area, in daylight, under clear weather conditions, at a non-junction with a posted speed limit of 55 mph; and closes in on a lead vehicle.

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

# 55 mph ≈ 24.6 m/s
param EGO_SPEED = Range(20, 24.6)
param LEAD_SPEED = globalParameters.EGO_SPEED * Uniform(0.5, 0.75)
param LEAD_DISTANCE = Range(40, 60)
param PASSING_DIST = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(leftLaneSec):
    laneChangeCompleted = False
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.PASSING_DIST) and not laneChangeCompleted:
        do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.EGO_SPEED)
        laneChangeCompleted = True

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, 'No lane sections with adjacent left lane in network.'

initLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = initLaneSec._laneToLeft

spawnPt = new OrientedPoint on initLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from spawnPt for globalParameters.LEAD_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawnPt,
    with regionContainedIn initLaneSec,
    with behavior EgoBehavior(leftLaneSec)

lead = new Car at leadSpawnPt,
    with regionContainedIn initLaneSec,
    with behavior LeadBehavior()

require (distance from ego to intersection) > 100
require (distance from lead to intersection) > 100

terminate when (distance from ego to spawnPt) > 200