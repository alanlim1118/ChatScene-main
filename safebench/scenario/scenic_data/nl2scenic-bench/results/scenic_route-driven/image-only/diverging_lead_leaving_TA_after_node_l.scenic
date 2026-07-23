"""Scenario Description:

A top-down schematic view depicts a four-way intersection where a blue vehicle at the bottom is identified as a diverging leading object executing a left turn. Blue arrows originating from the vehicle illustrate potential trajectories, including a solid arrow pointing straight ahead, a dashed arrow curving to the right, and a dashed arrow curving to the left, which corresponds to the object leaving the ego-traffic area to the left after the node. Purple indicators represent other traffic elements, specifically a solid purple line with a dot showing a vehicle from the top lane turning left across the intersection, along with dashed purple arrows denoting straight paths for traffic in the opposing and adjacent lanes.

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

EGO_INIT_DIST = [20, 25]

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego (blue vehicle) approaches from an incoming lane and may go straight, left, or right
egoManeuver = Uniform(*filter(lambda m:
        m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN),
        egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversary (purple vehicle) comes from the opposite direction and turns left across the intersection
advInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        Uniform(*filter(lambda m:
            m.type is ManeuverType.STRAIGHT,
            egoInitLane.maneuvers)
        ).reverseManeuvers)
    ).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
