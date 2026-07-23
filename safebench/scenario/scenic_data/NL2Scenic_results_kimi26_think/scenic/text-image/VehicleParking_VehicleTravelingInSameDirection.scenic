"""Scenario Description:

In an urban area during daylight with clear weather and a posted speed limit of 25 mph, a vehicle is depicted leaving a parked position by angling out from the curb into the traffic lane. The scene includes a designated handicapped parking spot marked with a yellow wheelchair symbol along the curb, and as the vehicle pulls out, it encounters another vehicle traveling in the same direction in a non-junction area.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

SPEED_LIMIT_MPS = 11.2          # 25 mph converted to m/s
PULL_OUT_DISTANCE = Range(20, 35)
PULL_OUT_TRIGGER = 12           # Distance at which the parked car begins to angle out

#################################
# AGENT BEHAVIORS               #
#################################

behavior PullOutBehavior(approaching_vehicle):
    # Wait until the approaching vehicle is close enough
    while distance from self to approaching_vehicle > PULL_OUT_TRIGGER:
        wait
    # Angle out from the curb into the traffic lane
    take SetThrottleAction(0.5), SetSteerAction(-0.4)
    do WaitBehavior() for 1.5 seconds
    take SetThrottleAction(0.4), SetSteerAction(0.0)
    do WaitBehavior() for 1.0 seconds
    do FollowLaneBehavior(target_speed=SPEED_LIMIT_MPS)

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=SPEED_LIMIT_MPS)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle traveling in the lane
ego = new Car with behavior EgoBehavior()

# Ensure non-junction area
require (distance to intersection) > 10

# Curb of the ego's current lane group
rightCurb = ego.laneGroup.curb
lane = Uniform(*ego.laneGroup.lanes)

# Designated handicapped parking spot ahead along the curb
parkSpot = new OrientedPoint on visible rightCurb ahead of ego by PULL_OUT_DISTANCE

# Parked car in the handicapped spot, which angles out as the ego approaches
parkedCar = new Car right of parkSpot by 1,
    facing along lane.orientation,
    with behavior PullOutBehavior(ego)

terminate after 30 seconds