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

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
TRUCK_MODEL = "vehicle.firetruck"  # Large red truck approximation

PEDESTRIAN_TRIGGER_DIST = 25       # Distance at which pedestrian starts crossing
TRUCK_SWERVE_TRIGGER_DIST = 20     # Distance at which truck begins swerving
EGO_SPEED = 8                      # Ego cruising speed (m/s)
PED_SPEED = 1.8                    # Pedestrian walking speed (m/s)
TRUCK_SPEED = 10                   # Truck initial speed (m/s)
SWERVE_DURATION = 3                # Duration of truck swerve maneuver (seconds)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveBehavior():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when collision:
        take SetThrottleAction(0), SetBrakeAction(1)

behavior PedestrianCrossFromLeft():
    while distance from self to ego > PEDESTRIAN_TRIGGER_DIST:
        wait
    # Cross from left to right relative to ego's heading
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(PED_SPEED)

behavior TruckSwerveRight(pedestrian_ref):
    try:
        do FollowLaneBehavior(target_speed=TRUCK_SPEED)
    interrupt when distance from self to pedestrian_ref < TRUCK_SWERVE_TRIGGER_DIST:
        # Swerve right into ego's lane to avoid pedestrian
        take SetSteeringAction(-0.8), SetThrottleAction(0.6)
        do WaitBehavior() for SWERVE_DURATION seconds
        # After swerve, continue in new lane or stop
        take SetSteeringAction(0), SetBrakeAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight multi-lane road segment
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 2 and s.isStraight, network.roads))
egoLane = Uniform(*filter(lambda l: l is not roadSegment.lanes[0], roadSegment.lanes))
leftLane = roadSegment.lanes[0]  # Leftmost lane for the truck

# Spawn points along the selected lanes
egoSpawnPt = new OrientedPoint in egoLane.centerline
truckSpawnPt = new OrientedPoint in leftLane.centerline,
    ahead of egoSpawnPt by Range(-5, 5)

# Pedestrian spawn point on the left sidewalk/curb ahead of ego
leftCurb = leftLane.leftEdge
pedSpawnRegion = leftCurb
pedSpawnPt = new OrientedPoint on visible pedSpawnRegion,
    ahead of egoSpawnPt by Range(30, 50)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set nighttime conditions
param timeOfDay = 'night'
param weather = 'ClearNight'

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EgoDriveBehavior()

truck = new Car at truckSpawnPt,
    with blueprint TRUCK_MODEL,
    with color (1.0, 0.0, 0.0),  # Red color
    with regionContainedIn None,
    with behavior TruckSwerveRight(pedestrian)

pedestrian = new Pedestrian at pedSpawnPt,
    facing -90 deg relative to pedSpawnPt,  # Facing right across the road
    with regionContainedIn None,
    with behavior PedestrianCrossFromLeft()

# Ensure proper initial spacing
require distance from ego to truck > 10
require distance from ego to pedestrian > 30
require distance from truck to pedestrian > 15

terminate after 45 seconds