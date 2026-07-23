"""Scenario Description:

The ego-vehicle encounters a vehicle merging into its lane from a highway on-ramp. The ego-vehicle must decelerate, brake or change lane to avoid a collision.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

param MERGE_SPEED = VerifaiRange(6, 10)
MERGE_DIST_AHEAD = VerifaiRange(30, 50)
SAFETY_DIST = VerifaiRange(12, 20)
CRASH_DIST = 4
TERM_DIST = 100
TERM_TIME = 15

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        try:
            fasterLaneSec = self.laneSection.fasterLane
            if fasterLaneSec is not None:
                do LaneChangeBehavior(
                    laneSectionToSwitch=fasterLaneSec,
                    target_speed=globalParameters.EGO_SPEED)
                do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
            else:
                take SetBrakeAction(globalParameters.EGO_BRAKE)
        interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
            terminate

behavior MergeBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.MERGE_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane that has a merging lane (on-ramp) feeding into it
mergeLanes = filter(lambda l: l.successor is not None and 
                    any(s.type == ManeuverType.MERGE for s in l.successor.maneuvers),
                    network.lanes)
egoLane = Uniform(*mergeLanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Find the merging lane that feeds into ego's lane
mergeManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.MERGE, egoLane.successor.maneuvers))
mergeLane = mergeManeuver.startLane
mergeSpawnPt = new OrientedPoint in mergeLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at mergeSpawnPt,
    with blueprint MODEL,
    with behavior MergeBehavior()

require MERGE_DIST_AHEAD[0] <= (distance from ego to adversary) <= MERGE_DIST_AHEAD[1]
require always (adversary.lane is not None)
terminate when (distance to egoSpawnPt) > TERM_DIST
terminate after TERM_TIME seconds