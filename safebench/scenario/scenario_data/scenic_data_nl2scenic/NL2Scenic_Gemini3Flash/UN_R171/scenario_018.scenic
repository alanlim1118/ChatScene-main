"""Scenario Description:
The ego vehicle follows a lead car on a multi-lane road. 
A stationary obstacle (either a motorcycle or a heavy truck) is positioned in the center of the lane ahead.
The lead car suddenly swerves into the adjacent lane to avoid the obstacle, 
leaving the ego vehicle to react to the stationary object.
"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_SPEED = 12
LEAD_CAR_SPEED = 12
LEAD_TO_OBSTACLE_DIST = Range(25, 35)
EGO_TO_LEAD_DIST = Range(10, 15)

# Threshold for the lead car to initiate the swerve
SWERVE_DISTANCE = 15 

# Models
obstacle_models = ['vehicle.yamaha.yzf', 'vehicle.tesla.cybertruck']
lead_car_models = ['vehicle.audi.tt', 'vehicle.bmw.grandtourer', 'vehicle.tesla.model3']

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadCarBehavior(target_speed, obstacle_obj, target_lane):
    """Drives forward and swerves when close to the obstacle."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to obstacle_obj) < SWERVE_DISTANCE:
        # Perform the lane change to avoid the obstacle
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=target_speed)
        # Continue driving in the new lane
        do FollowLaneBehavior(target_speed=target_speed)

behavior EgoBehavior(target_speed):
    """Follows lane and brakes if the obstacle (revealed by lead car) is too close."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, 10):
        # Emergency braking when the obstacle is revealed
        take SetBrakeAction(1.0)
        while True:
            take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane that has an adjacent lane to swerve into
laneSecsWithAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # Check if there's a lane to the left or right that is also in the same direction
        if laneSec.laneToLeft and laneSec.laneToLeft.isForward:
            laneSecsWithAdjacent.append((laneSec, laneSec.laneToLeft))
        elif laneSec.laneToRight and laneSec.laneToRight.isForward:
            laneSecsWithAdjacent.append((laneSec, laneSec.laneToRight))

assert len(laneSecsWithAdjacent) > 0, "No suitable multi-lane road found."

# Select a random lane section and its neighbor
selected_pair = Uniform(*laneSecsWithAdjacent)
start_lane_sec = selected_pair[0]
adjacent_lane_sec = selected_pair[1]

# Define spawn points
obstacle_spawn_pt = new OrientedPoint on start_lane_sec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. Stationary Obstacle (Motorcycle or Truck)
obstacle = new Vehicle at obstacle_spawn_pt,
    with blueprint Uniform(*obstacle_models),
    with behavior None # Stationary

# 2. Lead Car
lead_car = new Car following roadDirection from obstacle for -LEAD_TO_OBSTACLE_DIST,
    with blueprint Uniform(*lead_car_models),
    with behavior LeadCarBehavior(LEAD_CAR_SPEED, obstacle, adjacent_lane_sec)

# 3. Ego Vehicle
ego = new Car following roadDirection from lead_car for -EGO_TO_LEAD_DIST,
    with rolename 'hero',
    with behavior EgoBehavior(EGO_SPEED)

# Requirements
require (distance from ego to intersection) > 20
require (distance from obstacle to intersection) > 20

# Terminate after some time
terminate when simulation().currentTime > 150