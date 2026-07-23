"""Scenario Description:

In this aerial view of a straight, four-lane road flanked by green lawns and trees, the ego vehicle, an orange car positioned in the third lane from the left, is executing a lane change to the right lane. Its trajectory is marked by an orange line curving into the fourth lane, which is occupied by a white car traveling straight ahead. To the left, a yellow car in the first lane is merging right into the second lane, following a yellow trajectory path, while a teal car in the second lane and a purple car in the third lane proceed straight forward. Further up the road, a red car in the first lane, a green car in the second lane, and a blue car in the second lane are all traveling straight, indicated by their respective pink, green, and blue trajectory lines. A building with a brown roof is visible on the grassy area to the right of the roadway.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

OPT_SPEED_MAIN = Range(5, 10)
OPT_SPEED_FAR = Range(8, 12)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane section that belongs to a straight 4-lane forward road
laneSecFour = None
for lane in network.lanes:
    for ls in lane.sections:
        if (ls._laneToLeft is not None and ls._laneToLeft.isForward and
            ls._laneToRight is not None and ls._laneToRight.isForward and
            ls._laneToRight._laneToRight is not None and ls._laneToRight._laneToRight.isForward):
            laneSecFour = ls
            break
    if laneSecFour is not None:
        break

require laneSecFour is not None

# Identify the four lanes from left to right
lane2 = laneSecFour
lane1 = lane2._laneToLeft
lane3 = lane2._laneToRight
lane4 = lane3._laneToRight

# Project a common base point onto each lane centerline to align vehicles laterally
basePt = new OrientedPoint on lane2.centerline
pt1 = lane1.centerline.project(basePt.position)
pt2 = lane2.centerline.project(basePt.position)
pt3 = lane3.centerline.project(basePt.position)
pt4 = lane4.centerline.project(basePt.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego: orange car in 3rd lane, changing to 4th lane
ego = new Car at pt3,
    with color (1.0, 0.647, 0.0),
    with behavior LaneChangeBehavior(lane4, target_speed=OPT_SPEED_MAIN),
    facing roadDirection

# White car: 4th lane, straight ahead
whiteCar = new Car at follow roadDirection from pt4 for Range(15, 25),
    with color (1, 1, 1),
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED_MAIN),
    facing roadDirection

# Purple car: 3rd lane, straight (ahead of ego in the same lane)
purpleCar = new Car at follow roadDirection from pt3 for Range(25, 35),
    with color (0.5, 0, 0.5),
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED_MAIN),
    facing roadDirection

# Teal car: 2nd lane, straight
tealCar = new Car at pt2,
    with color (0, 0.5, 0.5),
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED_MAIN),
    facing roadDirection

# Yellow car: 1st lane, merging right into 2nd lane
yellowCar = new Car at pt1,
    with color (1, 1, 0),
    with behavior LaneChangeBehavior(lane2, target_speed=OPT_SPEED_MAIN),
    facing roadDirection

# Further up the road:
# Red car: 1st lane, straight
redCar = new Car at follow roadDirection from pt1 for Range(60, 80),
    with color (1, 0, 0),
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED_FAR),
    facing roadDirection

# Green car: 2nd lane, straight
greenCar = new Car at follow roadDirection from pt2 for Range(60, 75),
    with color (0, 1, 0),
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED_FAR),
    facing roadDirection

# Blue car: 2nd lane, straight (further ahead in same lane as green car)
blueCar = new Car at follow roadDirection from pt2 for Range(85, 100),
    with color (0, 0, 1),
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED_FAR),
    facing roadDirection