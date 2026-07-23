"""Scenario Description:

This top-down simulation view depicts a straight, multi-lane highway running vertically through a semi-rural environment featuring houses and trees on the left and a dense forest on the right. At the lower section of the road, a group of vehicles is clustered, including an orange ego vehicle and several adversary vehicles colored blue, green, yellow, and purple. The orange ego vehicle is executing a sequence of lane changes across the highway lanes towards the right, as indicated by the projected trajectory lines overlaid on the road surface. Simultaneously, the surrounding adversary vehicles are proceeding straight or performing their own intersecting lane changes, creating a dynamic traffic scenario where multiple agents navigate the multi-lane road concurrently.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = Range(8, 12)
param ADV_SPEED_STRAIGHT = Range(6, 10)
param ADV_SPEED_LANE_CHANGE = Range(7, 11)

param LANE_CHANGE_TRIGGER_DIST = Range(25, 35)
param SECOND_LANE_CHANGE_TRIGGER_DIST = Range(20, 30)

BLUE_BP = 'vehicle.lincoln.mkz_2017'
GREEN_BP = 'vehicle.lincoln.mkz_2017'
YELLOW_BP = 'vehicle.lincoln.mkz_2017'
PURPLE_BP = 'vehicle.lincoln.mkz_2017'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoSequentialLaneChangeBehavior(target_speed):
    """Ego performs sequential lane changes to the right."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to adv_blue) < globalParameters.LANE_CHANGE_TRIGGER_DIST:
        rightLane = self.laneSection._laneToRight
        if rightLane is not None:
            do LaneChangeBehavior(laneSectionToSwitch=rightLane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to adv_green) < globalParameters.SECOND_LANE_CHANGE_TRIGGER_DIST:
        rightLane = self.laneSection._laneToRight
        if rightLane is not None:
            do LaneChangeBehavior(laneSectionToSwitch=rightLane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)

behavior AdversaryStraightBehavior(target_speed):
    """Adversary drives straight in its lane."""
    do FollowLaneBehavior(target_speed=target_speed)

behavior AdversaryLaneChangeLeftBehavior(target_speed, trigger_dist):
    """Adversary changes lane to the left after approaching ego."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to ego) < trigger_dist:
        leftLane = self.laneSection._laneToLeft
        if leftLane is not None:
            do LaneChangeBehavior(laneSectionToSwitch=leftLane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)

behavior AdversaryLaneChangeRightBehavior(target_speed, trigger_dist):
    """Adversary changes lane to the right after approaching ego."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to ego) < trigger_dist:
        rightLane = self.laneSection._laneToRight
        if rightLane is not None:
            do LaneChangeBehavior(laneSectionToSwitch=rightLane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a multi-lane highway section with at least 3 lanes to the right
validStartLanes = []
for lane in network.lanes:
    for sec in lane.sections:
        if (sec.isForward 
            and sec._laneToRight is not None 
            and sec._laneToRight._laneToRight is not None):
            validStartLanes.append(sec)

require len(validStartLanes) > 0
egoLaneSec = Uniform(*validStartLanes)
firstRightLaneSec = egoLaneSec._laneToRight
secondRightLaneSec = firstRightLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Blue adversary: ahead in same lane as ego
blueSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(30, 45)

# Green adversary: ahead in first right lane
greenBasePt = firstRightLaneSec.centerline.project(egoSpawnPt.position)
greenSpawnPt = new OrientedPoint following roadDirection from greenBasePt for Range(25, 40)

# Yellow adversary: behind in second right lane, will change left
yellowBasePt = secondRightLaneSec.centerline.project(egoSpawnPt.position)
yellowSpawnPt = new OrientedPoint following roadDirection from yellowBasePt for Range(-15, -5)

# Purple adversary: ahead in second right lane, going straight
purpleBasePt = secondRightLaneSec.centerline.project(egoSpawnPt.position)
purpleSpawnPt = new OrientedPoint following roadDirection from purpleBasePt for Range(35, 55)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn egoLaneSec,
    with behavior EgoSequentialLaneChangeBehavior(globalParameters.EGO_SPEED),
    with color (1.0, 0.6, 0.0)  # Orange

adv_blue = new Car at blueSpawnPt,
    with blueprint BLUE_BP,
    with regionContainedIn egoLaneSec,
    with behavior AdversaryStraightBehavior(globalParameters.ADV_SPEED_STRAIGHT),
    with color (0.0, 0.0, 1.0)  # Blue

adv_green = new Car at greenSpawnPt,
    with blueprint GREEN_BP,
    with regionContainedIn firstRightLaneSec,
    with behavior AdversaryLaneChangeLeftBehavior(
        globalParameters.ADV_SPEED_LANE_CHANGE,
        Range(20, 30)
    ),
    with color (0.0, 0.8, 0.0)  # Green

adv_yellow = new Car at yellowSpawnPt,
    with blueprint YELLOW_BP,
    with regionContainedIn secondRightLaneSec,
    with behavior AdversaryLaneChangeRightBehavior(
        globalParameters.ADV_SPEED_LANE_CHANGE,
        Range(15, 25)
    ),
    with color (1.0, 1.0, 0.0)  # Yellow

adv_purple = new Car at purpleSpawnPt,
    with blueprint PURPLE_BP,
    with regionContainedIn secondRightLaneSec,
    with behavior AdversaryStraightBehavior(globalParameters.ADV_SPEED_STRAIGHT),
    with color (0.6, 0.0, 0.8)  # Purple

# Ensure sufficient distance from intersections for safe lane changes
require (distance to intersection) > 80
require (distance from adv_blue to intersection) > 80
require (distance from adv_green to intersection) > 80
require (distance from adv_yellow to intersection) > 80
require (distance from adv_purple to intersection) > 80

terminate when (distance from ego to adv_purple) > 100 or simulation().currentTime > 30