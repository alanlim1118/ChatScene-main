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

GREEN_MODEL = "vehicle.lincoln.mkz_2017"
BLUE_MODEL = "vehicle.tesla.model3"
PED_MODEL = "walker.pedestrian.0001"

param OPT_GREEN_SPEED = Range(5, 15)
param OPT_PED_SPEED = Range(1, 3)
param OPT_PED_START_DIST = Range(2, 4)
param OPT_GREEN_SPAWN_DIST = Range(20, 40)

OPT_LANE_WIDTH = 3.7
OPT_COLLISION_TO_BLUE_DIST = 5
BLUE_CAR_HALF_LENGTH = 2.3

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior WalkForwardBehavior(speed):
    take SetWalkingSpeedAction(speed)
    while True:
        wait

behavior GreenBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Anchor to an intersection to locate a straight road segment
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
# Select a straight incoming lane for the green vehicle (bottom lane, moving right)
greenLane = Uniform(*filter(lambda l: l.isStraight, intersection.incomingLanes))

# Collision point: where the pedestrian's path intersects the green vehicle's lane
collisionPt = new OrientedPoint in greenLane.centerline

# Green vehicle spawn point: behind the collision point in the bottom lane
greenSpawnPt = new OrientedPoint behind collisionPt by globalParameters.OPT_GREEN_SPAWN_DIST,
    with heading greenLane.centerline.heading

# Blue car longitudinal reference: offset so its front bumper is 5 m ahead of the collision point
blueLongRef = new OrientedPoint ahead of collisionPt by (OPT_COLLISION_TO_BLUE_DIST + BLUE_CAR_HALF_LENGTH),
    with heading greenLane.centerline.heading
# Shift laterally into the opposite (top) lane; car faces left (opposite to green lane heading)
blueSpawnPt = new OrientedPoint left of blueLongRef by OPT_LANE_WIDTH,
    with heading greenLane.centerline.heading + 180 deg

# Pedestrian spawn point: on the bottom edge (right side of road heading) walking upwards
pedSpawnPt = new OrientedPoint right of collisionPt by globalParameters.OPT_PED_START_DIST,
    with heading greenLane.centerline.heading + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

# Green vehicle traveling in the bottom lane towards the right
greenCar = new Car at greenSpawnPt,
    with regionContainedIn None,
    with blueprint GREEN_MODEL,
    with color (0, 1, 0),
    with behavior GreenBehavior(globalParameters.OPT_GREEN_SPEED)

# Blue stationary passenger car in the opposite top lane, facing left
blueCar = new Car at blueSpawnPt,
    with regionContainedIn None,
    with blueprint BLUE_MODEL,
    with color (0, 0, 1),
    with behavior WaitBehavior()
    # Driving beams (headlights) should be activated via simulator API

# Pedestrian crossing from the bottom edge upwards through the collision point
pedestrian = new Pedestrian at pedSpawnPt,
    with regionContainedIn None,
    with blueprint PED_MODEL,
    with behavior WalkForwardBehavior(globalParameters.OPT_PED_SPEED)

require 30 <= (distance from greenCar to intersection) <= 60
terminate when distance from greenCar to collisionPt > 60