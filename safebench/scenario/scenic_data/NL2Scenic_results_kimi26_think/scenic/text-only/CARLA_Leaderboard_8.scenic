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

param EGO_SPEED = VerifaiRange(10, 14)
param MERGE_SPEED = VerifaiRange(5, 8)
param EGO_DIST = VerifaiRange(30, 50)
param RAMP_DIST = VerifaiRange(10, 30)
param MERGE_DIST = VerifaiRange(15, 25)

TERM_DIST = 120

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to adversary) < globalParameters.MERGE_DIST:
        if self.laneSection.fasterLane is not None:
            do LaneChangeBehavior(
                    laneSectionToSwitch=self.laneSection.fasterLane,
                    target_speed=globalParameters.EGO_SPEED)
            do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
        else:
            do FollowLaneBehavior(target_speed=0)

behavior MergingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.MERGE_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Identify a merge lane fed by at least two predecessor lanes
mergeLane = Uniform(*filter(lambda l: len(l.predecessors) >= 2, network.lanes))
highwayApproachLane = Uniform(*mergeLane.predecessors)
rampLane = Uniform(*filter(lambda p: p is not highwayApproachLane, mergeLane.predecessors))

# Spawn points upstream of the merge point
egoSpawnPt = new OrientedPoint at (-globalParameters.EGO_DIST) @ 0 relative to highwayApproachLane.centerline.end
advSpawnPt = new OrientedPoint at (-globalParameters.RAMP_DIST) @ 0 relative to rampLane.centerline.end

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior MergingBehavior()

require ego.lane is not adversary.lane
terminate when (distance to egoSpawnPt) > TERM_DIST