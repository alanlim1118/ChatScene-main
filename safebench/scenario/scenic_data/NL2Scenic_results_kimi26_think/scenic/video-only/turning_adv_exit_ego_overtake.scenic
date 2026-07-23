"""Scenario Description:

The ego vehicle travels forward on a straight, two-lane residential road lined with uniform houses featuring red roofs under an overcast sky, following a grey sedan at a moderate speed. As the ego vehicle closes the distance, the lead grey sedan slows down and initiates a left turn to exit the road, crossing the lane. Simultaneously, the ego vehicle attempts to overtake the slowing sedan, but the turning vehicle cuts directly into its path, resulting in a sudden side-impact collision. After the impact, the ego vehicle slows significantly to a crawl, continuing forward on the now-clear road while passing pedestrians walking on the sidewalk to the right.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_SPEED = 10
SEDAN_SPEED = 10
TURN_TRIGGER_DIST = 20
OVERTAKE_TRIGGER_DIST = 20
PED_SPEED = 1.2

#################################
# AGENT BEHAVIORS               #
#################################

behavior SedanBehavior():
    do FollowLaneBehavior(target_speed=SEDAN_SPEED) until (distance from self to ego) < TURN_TRIGGER_DIST
    # Slow down and initiate a left turn, crossing into the oncoming lane
    while True:
        take SetThrottleAction(0.1), SetBrakeAction(0.3), SetSteerAction(-0.5)

behavior OvertakeBehavior():
    while True:
        take SetSteerAction(-0.35), SetThrottleAction(0.7)

behavior EgoBehavior(sedan):
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when (distance from self to sedan) < OVERTAKE_TRIGGER_DIST:
        do OvertakeBehavior() for 3 seconds
    # After impact, slow to a crawl and continue forward
    do FollowLaneBehavior(target_speed=2) for 10 seconds
    terminate

behavior WalkForward():
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(PED_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight two-lane residential road
road = Uniform(*filter(lambda r: len(r.lanes) == 2, network.roads))
lane = Uniform(*road.lanes)

start = new OrientedPoint on lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Lead grey sedan
sedan = new Car at start,
    with color "grey",
    with behavior SedanBehavior()

# Ego vehicle starts behind the sedan
egoStart = new OrientedPoint following start.heading from start for -30
ego = new Car at egoStart,
    with behavior EgoBehavior(sedan)

# Pedestrians walking on the right sidewalk, ahead of the collision area
aheadPed1 = new OrientedPoint ahead of sedan by 60
ped1 = new Pedestrian right of aheadPed1 by 4,
    facing start.heading,
    with behavior WalkForward()

aheadPed2 = new OrientedPoint ahead of sedan by 72
ped2 = new Pedestrian right of aheadPed2 by 4,
    facing start.heading,
    with behavior WalkForward()

terminate after 30 seconds