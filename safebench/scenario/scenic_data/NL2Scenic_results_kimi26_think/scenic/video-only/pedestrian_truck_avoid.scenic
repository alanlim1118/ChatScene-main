"""Scenario Description:

The ego vehicle is driving on a multi-lane urban road at night when a pedestrian suddenly crosses the street from the left side. A large red truck traveling in the adjacent left lane swerves sharply to the right to avoid hitting the pedestrian, cutting directly into the ego vehicle's lane. This panic maneuver causes the truck to collide with the side of the ego vehicle, forcing the car towards the right curb and roadside vegetation.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
param weather = "ClearNight"

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_TRUCK_SPEED = Range(5, 10)
param OPT_PED_SPEED = Range(1.5, 3.0)

PEDESTRIAN_TRIGGER_DIST = 25
SWERVE_TRIGGER_DIST = 10
LANE_WIDTH = 3.6

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(globalParameters.OPT_EGO_SPEED)

behavior CrossBehavior():
    while distance from self to truck > PEDESTRIAN_TRIGGER_DIST:
        wait
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(globalParameters.OPT_PED_SPEED)

behavior TruckBehavior():
    try:
        do FollowLaneBehavior(globalParameters.OPT_TRUCK_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, SWERVE_TRIGGER_DIST):
        while True:
            take SetSteerAction(1.0), SetThrottleAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a multi-lane urban road with at least 2 lanes in the same direction
laneGroup = Uniform(*filter(lambda g: len(g.lanes) >= 2, network.laneGroups))
leftLane = laneGroup.lanes[0]
rightLane = laneGroup.lanes[1]

# Spawn ego in the right lane
egoSpawn = new OrientedPoint on rightLane.centerline
ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Truck in the adjacent left lane, alongside the ego
truck = new Car left of ego by LANE_WIDTH,
    with blueprint TRUCK_MODEL,
    with color [255, 0, 0],
    with behavior TruckBehavior()

# Pedestrian on the left sidewalk, ahead of the truck
pedSpawn = new OrientedPoint ahead of truck by 20
pedestrian = new Pedestrian left of pedSpawn by 2.0,
    facing (egoSpawn.heading - 90 deg),
    with behavior CrossBehavior()

terminate after 30 seconds