"""Scenario Description:

Vehicle is turning right in an urban area, in daylight, under clear weather conditions, 
at a signalized intersection with a posted speed limit of 35 mph; and turns into the 
same direction of another vehicle crossing straight initially from a lateral direction.

- Town: Town05 (Urban grid with multi-lane intersections)
- Weather: ClearNoon
- Speed: 35 mph (~15.6 m/s)
- Intersection: Signalized 4-way intersection
- Agents: Ego vehicle (turning right), Lead vehicle (crossing straight)
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

# 35 mph to m/s conversion
TARGET_SPEED = 15.6

# Distances from the intersection entry point
EGO_DIST = Range(18, 22)
OTHER_DIST = Range(8, 12)

# Weather setup
param weather = 'ClearNoon'

# Models
EGO_MODEL = 'vehicle.audi.tt'
OTHER_MODEL = 'vehicle.lincoln.mkz_2017'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoTurningBehavior(trajectory, target_speed):
    """Ego follows a trajectory but brakes if the lateral car is in the way."""
    try:
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
    interrupt when withinDistanceToAnyCars(self, 8):
        take SetBrakeAction(1.0)

behavior CrossingBehavior(trajectory, target_speed):
    """Lateral car crosses straight through the intersection."""
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

#################################
# MONITOR                       #
#################################

monitor TrafficLightManager:
    """Ensure the traffic lights stay green for the duration of the maneuver."""
    freezeTrafficLights()
    while True:
        # Set lights for both roads to green to ensure flow
        setClosestTrafficLightStatus(ego, 'green', distance=100)
        setClosestTrafficLightStatus(other, 'green', distance=100)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Filter for a suitable signalized 4-way intersection
intersections = filter(lambda i: i.isSignalized and i.is4Way, network.intersections)
intersec = Uniform(*intersections)

# 2. Find a straight maneuver for the 'other' vehicle
straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersec.maneuvers)

# 3. Find a corresponding right turn for 'ego' that ends in the same lane
# and starts from a lateral (perpendicular) road.
valid_pairs = []
for sm in straight_maneuvers:
    rights = filter(lambda rm: rm.type == ManeuverType.RIGHT_TURN 
                    and rm.endLane == sm.endLane 
                    and rm.startLane.road != sm.startLane.road, 
                    intersec.maneuvers)
    for rm in rights:
        valid_pairs.append((sm, rm))

# Ensure we have a valid configuration
require len(valid_pairs) > 0
selected_pair = Uniform(*valid_pairs)
otherManeuver, egoManeuver = selected_pair

# 4. Define Trajectories
otherTrajectory = [otherManeuver.startLane, otherManeuver.connectingLane, otherManeuver.endLane]
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 5. Define Spawn Points
# Centerline[-1] is the point where the lane enters the intersection
other_entry_pt = otherManeuver.startLane.centerline[-1]
ego_entry_pt = egoManeuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the other vehicle (the straight crosser) closer to the intersection
other = new Car following roadDirection from other_entry_pt for -OTHER_DIST,
    with blueprint OTHER_MODEL,
    with behavior CrossingBehavior(otherTrajectory, TARGET_SPEED)

# Spawn the ego vehicle further back so it follows the other vehicle into the lane
ego = new Car following roadDirection from ego_entry_pt for -EGO_DIST,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoTurningBehavior(egoTrajectory, TARGET_SPEED)

# Run the traffic light monitor
require monitor TrafficLightManager()

# Termination condition
terminate when (distance from ego to ego_entry_pt) > 50