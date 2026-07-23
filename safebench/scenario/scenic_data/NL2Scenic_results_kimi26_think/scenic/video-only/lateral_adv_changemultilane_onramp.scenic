"""Scenario Description:

The ego vehicle travels forward on a multi-lane roadway under clear, sunny skies, approaching a junction marked by an overpass with a red banner and blue directional signs for the Taiyuan West Ring Expressway. As the lanes diverge, a white sedan in the right lane abruptly swerves left, aggressively crossing the gore area and solid white lines to force its way back onto the main carriageway. This vehicle cuts directly across the ego vehicle's path, resulting in a sudden side-impact collision as it attempts to merge in front of the ego car just before the underpass.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town06'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town06'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.a2'

param OPT_EGO_SPEED = Range(10, 14)
param OPT_ADV_SPEED = Range(6, 9)
param OPT_SWERVE_DIST = Range(12, 20)
param OPT_BRAKE_DIST = Range(2, 4)
param OPT_EGO_BRAKE = Range(0.8, 1.0)
param OPT_GEO_X_DISTANCE = Range(8, 16)
param OPT_GEO_Y_DISTANCE = Range(-5, -3)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(globalParameters.OPT_EGO_BRAKE)

behavior AdvBehavior(target_lane, ego_vehicle):
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego_vehicle < globalParameters.OPT_SWERVE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find forward lane sections that have a right lane going the same direction
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToRight is not None and
            laneSec._laneToRight.isForward
        ):
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
advLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE
advSpawnPt = new OrientedPoint at egoSpawnPt offset along egoSpawnPt.heading by SHIFT

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(egoLaneSec, ego)

terminate when (distance from ego to egoSpawnPt) > 100