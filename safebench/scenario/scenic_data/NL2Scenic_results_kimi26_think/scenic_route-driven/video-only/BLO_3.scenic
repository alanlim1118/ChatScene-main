"""Scenario Description:

The ego vehicle, represented by a green bounding box, is traveling along a straight multi-lane road section positioned between two four-way intersections. Its forward progress is impeded by a dense cluster of adversary vehicles, indicated by yellow bounding boxes, which are stopped or moving slowly directly ahead and in the adjacent lanes. This accumulation of traffic creates a bottleneck that completely obstructs the ego vehicle's path, effectively forcing it to remain stationary behind the congestion.

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
ADV_MODEL = 'vehicle.lincoln.mkz_2017'

param ADV_SPEED = Range(0, 2)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

# Align base points in adjacent lanes with the ego
leftBasePt = new OrientedPoint in leftLaneSec.centerline
require distance from leftBasePt to egoSpawnPt < 4.0

rightBasePt = new OrientedPoint in rightLaneSec.centerline
require distance from rightBasePt to egoSpawnPt < 4.0

# Adversary cluster points ahead of the ego
advSame1Pt = new OrientedPoint following roadDirection from egoSpawnPt for Range(20, 30)
advSame2Pt = new OrientedPoint following roadDirection from egoSpawnPt for Range(25, 35)

advLeft1Pt = new OrientedPoint following roadDirection from leftBasePt for Range(20, 30)
advLeft2Pt = new OrientedPoint following roadDirection from leftBasePt for Range(25, 35)

advRight1Pt = new OrientedPoint following roadDirection from rightBasePt for Range(20, 30)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL

adversary1 = new Car at advSame1Pt,
    with blueprint ADV_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary2 = new Car at advSame2Pt,
    with blueprint ADV_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary3 = new Car at advLeft1Pt,
    with blueprint ADV_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary4 = new Car at advLeft2Pt,
    with blueprint ADV_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary5 = new Car at advRight1Pt,
    with blueprint ADV_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

# Ensure the ego is on a straight section between intersections (not too close to any 4-way intersection)
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
require 20 <= (distance to intersection) <= 80

terminate after 20 seconds
