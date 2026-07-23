"""Scenario Description:

In this aerial view of a straight, four-lane road flanked by green lawns and trees, the ego vehicle, an orange car positioned in the third lane from the left, is executing a lane change to the right lane. Its trajectory is marked by an orange line curving into the fourth lane, which is occupied by a white car traveling straight ahead. To the left, a yellow car in the first lane is merging right into the second lane, following a yellow trajectory path, while a teal car in the second lane and a purple car in the third lane proceed straight forward. Further up the road, a red car in the first lane, a green car in the second lane, and a blue car in the second lane are all traveling straight, indicated by their respective pink, green, and blue trajectory lines. A building with a brown roof is visible on the grassy area to the right of the roadway.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
WHITE_CAR_MODEL = "vehicle.tesla.model3"
YELLOW_CAR_MODEL = "vehicle.audi.tt"
TEAL_CAR_MODEL = "vehicle.volkswagen.t2"
PURPLE_CAR_MODEL = "vehicle.nissan.patrol"
RED_CAR_MODEL = "vehicle.dodge.charger_police"
GREEN_CAR_MODEL = "vehicle.jeep.wrangler_rubicon"
BLUE_CAR_MODEL = "vehicle.bmw.grandtourer"

#################################
# AGENT BEHAVIORS               #
#################################

behavior LaneChangeRightBehavior(target_speed=10):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to self.laneSection._laneToRight.centerline) < 2:
        take SetTurnAction('right')
        do FollowLaneBehavior(target_speed=target_speed) for 3 seconds
        take SetTurnAction(None)
        do FollowLaneBehavior(target_speed=target_speed)

behavior MergeRightBehavior(target_speed=10):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to self.laneSection._laneToRight.centerline) < 3:
        take SetTurnAction('right')
        do FollowLaneBehavior(target_speed=target_speed) for 4 seconds
        take SetTurnAction(None)
        do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight four-lane road section
fourLaneSections = []
for lane in network.lanes:
    if lane.isForward:
        sec = lane.sections[0] if lane.sections else None
        if sec is not None:
            # Check if there are at least 3 lanes to the right (making it 4+ lanes total)
            current = sec
            count = 1
            while current._laneToRight is not None and current._laneToRight.isForward:
                current = current._laneToRight
                count += 1
            if count >= 4:
                fourLaneSections.append(sec)

require len(fourLaneSections) > 0
baseLaneSec = Uniform(*fourLaneSections)

# Define lane sections from left to right: lane1 (leftmost), lane2, lane3 (ego start), lane4 (rightmost)
lane1Sec = baseLaneSec
lane2Sec = lane1Sec._laneToRight
lane3Sec = lane2Sec._laneToRight
lane4Sec = lane3Sec._laneToRight

require lane4Sec is not None

# Reference point on lane3 centerline for ego placement
egoRefPt = new OrientedPoint on lane3Sec.centerline

# Points further ahead for distant vehicles
aheadRefPt = follow roadDirection from egoRefPt for Range(60, 90)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle: orange car in lane3, changing to lane4
ego = new Car at egoRefPt,
    facing roadDirection,
    with blueprint EGO_MODEL,
    with color (1.0, 0.6, 0.0),
    with behavior LaneChangeRightBehavior(target_speed=Range(8, 12))

# White car in lane4 traveling straight (in front of ego's target lane)
whiteCar = new Car at (follow roadDirection from egoRefPt for Range(15, 30)),
    with regionContainedIn lane4Sec,
    facing roadDirection,
    with blueprint WHITE_CAR_MODEL,
    with color (1.0, 1.0, 1.0),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Yellow car in lane1 merging right into lane2
yellowCar = new Car at (follow roadDirection from egoRefPt for Range(-5, 10)),
    with regionContainedIn lane1Sec,
    facing roadDirection,
    with blueprint YELLOW_CAR_MODEL,
    with color (1.0, 0.9, 0.0),
    with behavior MergeRightBehavior(target_speed=Range(8, 12))

# Teal car in lane2 proceeding straight
tealCar = new Car at (follow roadDirection from egoRefPt for Range(5, 20)),
    with regionContainedIn lane2Sec,
    facing roadDirection,
    with blueprint TEAL_CAR_MODEL,
    with color (0.0, 0.7, 0.7),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Purple car in lane3 proceeding straight (behind or near ego)
purpleCar = new Car at (follow roadDirection from egoRefPt for Range(-20, -5)),
    with regionContainedIn lane3Sec,
    facing roadDirection,
    with blueprint PURPLE_CAR_MODEL,
    with color (0.6, 0.0, 0.8),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Red car in lane1 further ahead, traveling straight
redCar = new Car at (follow roadDirection from aheadRefPt for Range(0, 15)),
    with regionContainedIn lane1Sec,
    facing roadDirection,
    with blueprint RED_CAR_MODEL,
    with color (1.0, 0.0, 0.0),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Green car in lane2 further ahead, traveling straight
greenCar = new Car at (follow roadDirection from aheadRefPt for Range(5, 20)),
    with regionContainedIn lane2Sec,
    facing roadDirection,
    with blueprint GREEN_CAR_MODEL,
    with color (0.0, 0.8, 0.0),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Blue car in lane2 further ahead (ahead of green car), traveling straight
blueCar = new Car at (follow roadDirection from aheadRefPt for Range(25, 40)),
    with regionContainedIn lane2Sec,
    facing roadDirection,
    with blueprint BLUE_CAR_MODEL,
    with color (0.0, 0.0, 1.0),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Building with brown roof on grassy area to the right of the roadway
buildingSpot = new OrientedPoint right of egoRefPt by Range(15, 25),
    facing roadDirection
new Object at buildingSpot,
    with shape BoxShape(width=Range(5, 10), length=Range(5, 10), height=Range(4, 8)),
    with color (0.55, 0.27, 0.07)

# Termination condition
terminate when distance from ego to whiteCar > 80 or simulation().currentTime > 30