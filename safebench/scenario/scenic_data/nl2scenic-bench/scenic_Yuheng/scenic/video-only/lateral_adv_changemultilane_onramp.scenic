"""Scenario Description:

The ego vehicle travels forward on a multi-lane roadway under clear, sunny skies, approaching a junction marked by an overpass with a red banner and blue directional signs for the Taiyuan West Ring Expressway. As the lanes diverge, a white sedan in the right lane abruptly swerves left, aggressively crossing the gore area and solid white lines to force its way back onto the main carriageway. This vehicle cuts directly across the ego vehicle's path, resulting in a sudden side-impact collision as it attempts to merge in front of the ego car just before the underpass.

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
ADV_MODEL = 'vehicle.tesla.model3'  # White sedan approximation

param EGO_SPEED = VerifaiRange(8, 12)
param ADV_SPEED_INITIAL = VerifaiRange(6, 9)
param ADV_SPEED_MERGE = VerifaiRange(10, 14)
param MERGE_TRIGGER_DIST = VerifaiRange(25, 40)
param BRAKE_DIST = VerifaiRange(8, 15)
param CRASH_DIST = 3.0
param TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.CRASH_DIST):
        terminate

behavior AggressiveMergeBehavior(trigger_dist, initial_speed, merge_speed):
    # Initially follow the right lane at moderate speed
    do FollowLaneBehavior(target_speed=initial_speed) until (distance from self to ego <= trigger_dist)
    # Abruptly swerve left into ego's lane (crossing gore/solid lines)
    do LaneChangeBehavior(
        laneSectionToSwitch=self.laneSection._laneToLeft,
        is_oppositeTraffic=False,
        target_speed=merge_speed
    )
    # Continue in ego's lane after merge attempt
    do FollowLaneBehavior(target_speed=merge_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a road section with at least two forward lanes where right lane has a left neighbor
multiLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if (
            sec.isForward and
            sec._laneToLeft is not None and
            sec._laneToLeft.isForward
        ):
            multiLaneSections.append(sec)

egoLaneSec = Uniform(*multiLaneSections)
rightLaneSec = egoLaneSec._laneToRight if egoLaneSec._laneToRight is not None else egoLaneSec

# Ego spawns in the left/main lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversary spawns in the right lane, ahead of ego but within merge trigger range
advOffsetAlongRoad = Range(15, 30)
advSpawnBase = new OrientedPoint following roadDirection from egoSpawnPt for advOffsetAlongRoad
advSpawnPt = new OrientedPoint at advSpawnBase offset laterally by -egoLaneSec.width

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set clear sunny weather
param weather = Weather(
    cloudiness=0,
    precipitation=0,
    wetness=0,
    sunAltitudeAngle=70
)

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (1.0, 1.0, 1.0),  # White sedan
    with behavior AggressiveMergeBehavior(
        globalParameters.MERGE_TRIGGER_DIST,
        globalParameters.ADV_SPEED_INITIAL,
        globalParameters.ADV_SPEED_MERGE
    )

require distance from ego to adversary >= 10
require distance from ego to adversary <= 50
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST