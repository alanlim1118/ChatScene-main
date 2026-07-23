"""Scenario Description:

On a straight, unlit two-way road with two lanes separated by double yellow lines, a green vehicle travels in the bottom lane towards the right, approaching a pedestrian crossing from the bottom edge of the road upwards. In the opposite top lane, a blue stationary passenger car faces left with its driving beams activated, projecting a wide cone of light across the road surface. An estimated collision point is marked where the pedestrian's path intersects the lane of the moving green vehicle, and a dimension line indicates a longitudinal distance of 5 meters between this collision point and the front of the stationary blue car.

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

GREEN_CAR_MODEL = "vehicle.tesla.model3"
BLUE_CAR_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_GREEN_SPEED = Range(3, 6)
param OPT_PED_SPEED = Range(1, 3)
param OPT_PED_START_DISTANCE = Range(15, 25)
param COLLISION_TO_BLUE_DIST = 5  # Longitudinal distance from collision point to front of blue car

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior GreenCarBehavior():
    try:
        do FollowLaneBehavior(globalParameters.OPT_GREEN_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, 5)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        terminate

behavior PedestrianCrossingBehavior(target_speed):
    take SetWalkingSpeedAction(target_speed)
    do CrossingBehavior(None, target_speed, 30)
    take SetWalkingSpeedAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight road segment suitable for the scenario
straightRoads = filter(lambda r: len(r.lanes) >= 2, network.roads)
road = Uniform(*straightRoads)

# Identify the two lanes: bottom lane (green car direction) and top lane (blue car direction)
# Assuming standard right-hand traffic: bottom lane goes right, top lane goes left
bottomLane = Uniform(*filter(lambda l: l.isForward, road.lanes))
topLane = Uniform(*filter(lambda l: not l.isForward and l.road is bottomLane.road, road.lanes))

# Define spawn points along the bottom lane centerline
greenSpawnPt = new OrientedPoint in bottomLane.centerline

# Collision point is ahead of the green car along the bottom lane
collisionPoint = new OrientedPoint following bottomLane.orientation from greenSpawnPt for globalParameters.OPT_PED_START_DISTANCE

# Blue car is positioned in the top lane such that its front is 5m longitudinally from the collision point
# Since blue car faces left (opposite direction), its front points toward decreasing lane parameter
# Place blue car so that distance from collisionPoint to blueCar.front is 5m along the road axis
blueCarRefPt = new OrientedPoint at collisionPoint,
    with heading topLane.centerline.heading
blueSpawnPt = new OrientedPoint following topLane.orientation from blueCarRefPt for -COLLISION_TO_BLUE_DIST

# Pedestrian starts at the bottom edge of the road and crosses upward through the collision point
pedStartPt = new OrientedPoint at collisionPoint,
    with heading bottomLane.centerline.heading + 90 deg  # Perpendicular, crossing from bottom to top
# Offset pedestrian start to be at the bottom edge of the road
pedSpawnPt = new OrientedPoint behind pedStartPt by (bottomLane.width / 2 + 1)

#################################
# SCENARIO SPECIFICATION        #
#################################

greenCar = new Car at greenSpawnPt,
    with regionContainedIn None,
    with blueprint GREEN_CAR_MODEL,
    with color "Green",
    with behavior GreenCarBehavior()

blueCar = new Car at blueSpawnPt,
    with regionContainedIn None,
    with blueprint BLUE_CAR_MODEL,
    with color "Blue",
    with headlights On,
    with behavior WaitBehavior()

pedestrian = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior(globalParameters.OPT_PED_SPEED)

# Ensure the collision point lies within the bottom lane
require collisionPoint in bottomLane
# Ensure blue car is properly placed in the top lane
require blueSpawnPt in topLane
# Ensure reasonable spacing
require distance from greenCar to collisionPoint >= 10