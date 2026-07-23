"""Scenario Description:

Ego vehicle (blue) travels straight in the center lane of a multi-lane road. An adversary vehicle (pink) is positioned in the adjacent left lane slightly ahead, attempts to merge into the center lane, but aborts the maneuver and returns to the left lane, while the ego vehicle maintains its course.

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

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param ADV_SPEED = VerifaiRange(6, 9)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 100

LANE_WIDTH = 3.6
AHEAD_DIST = [8, 18]

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(ego_lane):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, lane=ego_lane)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior AbortMergeBehavior(ego_lane, adv_lane):
    # Drive in left lane
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, lane=adv_lane) for 3
    # Attempt to merge into center lane
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, lane=ego_lane) for 2
    # Abort merge and return to left lane
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, lane=adv_lane)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a forward lane that has a left neighbor traveling in the same direction
egoLane = Uniform(*filter(lambda l: l.isForward and l.leftLane is not None and l.leftLane.isForward, network.lanes))
advLane = egoLane.leftLane

egoSpawnPt = new OrientedPoint in egoLane.centerline
advSpawnPt = new OrientedPoint at egoSpawnPt offset by (-LANE_WIDTH, Uniform(*AHEAD_DIST))

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoLane)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AbortMergeBehavior(egoLane, advLane)

terminate when (distance to egoSpawnPt) > TERM_DIST