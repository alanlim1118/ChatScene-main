"""Scenario Description:

A vehicle is depicted leaving a parked position in a rural area at night under clear weather conditions, angling out from a parking space adjacent to a designated handicapped spot marked with a wheelchair symbol. An arrow indicates the car's forward trajectory as it moves into the lane, where it encounters a dog standing directly in its path at a non-junction area. The scene is presented as a top-down schematic diagram showing the vehicle, the animal, and the road markings including vertical lines separating parking bays.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_DOG_DISTANCE = Range(10, 20)
param OPT_BRAKE_DIST = Range(4, 7)
param OPT_PARK_OFFSET = Range(4, 6)
param OPT_BAY_SEPARATION = Range(3, 4)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior PullOutAndBrakeBehavior(obstacle, brake_dist):
    try:
        while True:
            take SetThrottleAction(0.5)
    interrupt when (distance from self to obstacle <= brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight, non-junction lane in a rural area
lane = Uniform(*filter(lambda l: l.isForward and not l.isIntersection and l.length > 60, network.lanes))

# Reference point in the lane where the dog will be placed
laneRefPt = new OrientedPoint on lane.centerline

# Dog stands in the lane ahead, directly in the ego's forward path
dogPt = new OrientedPoint ahead of laneRefPt by globalParameters.OPT_DOG_DISTANCE

# Dog agent (represents the animal in the path)
Dog = new Pedestrian at dogPt,
    with heading lane.orientation + 180 deg,
    with behavior WaitBehavior(),
    with regionContainedIn None

# Ego spawn: in a parking bay to the right of the lane, angled out toward the lane
# The heading is angled to represent pulling out from a perpendicular parking bay
egoSpawnPt = new OrientedPoint right of laneRefPt by globalParameters.OPT_PARK_OFFSET,
    with heading lane.orientation + Range(25, 45) deg

# Adjacent handicapped spot: offset along the road direction to sit next to the ego's bay
handicapPt = new OrientedPoint at egoSpawnPt offset along lane.orientation by globalParameters.OPT_BAY_SEPARATION,
    with heading lane.orientation + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle leaving parked position, angling out into the lane
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior PullOutAndBrakeBehavior(Dog, globalParameters.OPT_BRAKE_DIST),
    with regionContainedIn None

# Parked car in the adjacent handicapped spot (represents the designated handicapped space)
handicapCar = new Car at handicapPt,
    with heading handicapPt.heading,
    with regionContainedIn None

# Ensure the dog is within a reasonable distance ahead of the pull-out point
require distance from ego to Dog > 5
require distance from ego to Dog < 25