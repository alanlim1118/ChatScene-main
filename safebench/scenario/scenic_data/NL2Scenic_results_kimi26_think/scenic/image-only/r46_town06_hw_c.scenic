"""Scenario Description:

This top-down simulation view depicts a straight, multi-lane highway running vertically through a semi-rural environment featuring houses and trees on the left and a dense forest on the right. At the lower section of the road, a group of vehicles is clustered, including an orange ego vehicle and several adversary vehicles colored blue, green, yellow, and purple. The orange ego vehicle is executing a sequence of lane changes across the highway lanes towards the right, as indicated by the projected trajectory lines overlaid on the road surface. Simultaneously, the surrounding adversary vehicles are proceeding straight or performing their own intersecting lane changes, creating a dynamic traffic scenario where multiple agents navigate the multi-lane road concurrently.

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

param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(6, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed):
    # Sequence of lane changes towards the right across multiple lanes
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=target_speed)
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

behavior AdvStraightBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior AdvChangeLeftBehavior(target_speed):
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

behavior AdvChangeRightBehavior(target_speed):
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a forward lane section with at least two lanes to the right
validLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToLeft is not None and 
            laneSec._laneToRight is not None and
            laneSec._laneToRight._laneToRight is not None):
            validLaneSecs.append(laneSec)

egoLaneSec = Uniform(*validLaneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Define adjacent lanes relative to ego spawn
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec1 = egoLaneSec._laneToRight
rightLaneSec2 = rightLaneSec1._laneToRight

# Project ego position onto adjacent lane centerlines for clustered spawning
leftProj = leftLaneSec.centerline.project(egoSpawnPt.position)
right2Proj = rightLaneSec2.centerline.project(egoSpawnPt.position)

# Clustered spawn points near the lower section of the road
adv_blue_spawn = new OrientedPoint following roadDirection from right2Proj for Range(5, 15)
adv_green_spawn = new OrientedPoint following roadDirection from right2Proj for Range(-15, -5)
adv_yellow_spawn = new OrientedPoint following roadDirection from leftProj for Range(5, 15)
adv_purple_spawn = new OrientedPoint following roadDirection from leftProj for Range(-15, -5)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Orange ego vehicle: starts middle, changes right twice
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with color (255, 165, 0),
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# Blue adversary: rightmost lane, ahead, proceeding straight
adv_blue = new Car at adv_blue_spawn,
    with regionContainedIn rightLaneSec2,
    with blueprint MODEL,
    with color (0, 0, 255),
    with behavior AdvStraightBehavior(globalParameters.ADV_SPEED)

# Green adversary: rightmost lane, behind, performing intersecting lane change to the left
adv_green = new Car at adv_green_spawn,
    with regionContainedIn rightLaneSec2,
    with blueprint MODEL,
    with color (0, 255, 0),
    with behavior AdvChangeLeftBehavior(globalParameters.ADV_SPEED)

# Yellow adversary: left lane, ahead, proceeding straight
adv_yellow = new Car at adv_yellow_spawn,
    with regionContainedIn leftLaneSec,
    with blueprint MODEL,
    with color (255, 255, 0),
    with behavior AdvStraightBehavior(globalParameters.ADV_SPEED)

# Purple adversary: left lane, behind, performing intersecting lane change to the right
adv_purple = new Car at adv_purple_spawn,
    with regionContainedIn leftLaneSec,
    with blueprint MODEL,
    with color (128, 0, 128),
    with behavior AdvChangeRightBehavior(globalParameters.ADV_SPEED)

# Keep scenario away from intersections to maintain highway context
require (distance to intersection) > 50
terminate when (distance from ego to adv_blue) > 200