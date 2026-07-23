"""Scenario Description:

The ego vehicle proceeds forward on a narrow, unlit rural village road at night, surrounded by buildings on both sides. An oncoming truck with bright headlights approaches and passes the ego vehicle on the left. As the truck passes, a pedestrian unexpectedly emerges from behind it, crossing the road from left to right directly into the ego vehicle's path. The ego vehicle attempts to brake suddenly but is unable to stop in time due to the limited visibility and short reaction distance, leading to a collision with the pedestrian. Afterward, the ego vehicle comes to a halt near a white SUV parked on the left side of the road.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town06'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town06'
model scenic.domains.driving.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
TRUCK_MODEL = "vehicle.carlamotors.firetruck"
PARKED_SUV_MODEL = "vehicle.tesla.model3"

PEDESTRIAN_TRIGGER_DISTANCE = 18     # Distance at which pedestrian begins crossing (when truck is passing)
BRAKE_TRIGGER_DISTANCE = 12          # Distance at which ego begins braking
EGO_TO_TRUCK_MIN_DIST = 40           # Initial distance between ego and oncoming truck
TRUCK_SPEED = 12                     # Speed of oncoming truck
PEDESTRIAN_SPEED = 1.8               # Walking speed of pedestrian
PARKED_SUV_OFFSET = 1.5              # Offset of parked SUV from left curb

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveAndBrake():
    try:
        do FollowLaneBehavior(target_speed=8)
    interrupt when withinDistanceToAnyPedestrians(self, BRAKE_TRIGGER_DISTANCE):
        take SetThrottleAction(0), SetBrakeAction(1)
        wait

behavior OncomingTruckBehavior():
    do FollowLaneBehavior(target_speed=TRUCK_SPEED)

behavior PedestrianCrossFromBehindTruck(truck_ref):
    # Wait until the truck has passed close to ego (i.e., truck is near or just past ego)
    while distance from self to ego > PEDESTRIAN_TRIGGER_DISTANCE:
        wait
    # Cross from left to right relative to ego's heading
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(PEDESTRIAN_SPEED)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Place ego on a straight rural road segment
egoLane = Uniform(*filter(lambda l: l.isForward and not l.isIntersection, network.lanes))
egoSpawn = new OrientedPoint in egoLane.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoDriveAndBrake(),
    with regionContainedIn None

# Oncoming truck in opposing lane
opposingLane = egoLane.oppositeLane
require opposingLane is not None
truckSpawn = new OrientedPoint in opposingLane.centerline,
    facing opposite ego.heading

require distance from ego to truckSpawn >= EGO_TO_TRUCK_MIN_DIST

oncomingTruck = new Car at truckSpawn,
    with blueprint TRUCK_MODEL,
    with behavior OncomingTruckBehavior(),
    with regionContainedIn None

# Pedestrian initially hidden behind the truck on the left side of ego's road
leftCurb = egoLane.leftEdge
pedSpawnBase = new OrientedPoint on visible leftCurb,
    ahead of truckSpawn by -5  # Slightly behind truck's initial position along road

pedestrian = new Pedestrian left of pedSpawnBase by 0.5,
    facing ego.heading + 90 deg,   # Facing right across the road (left-to-right from ego perspective)
    with behavior PedestrianCrossFromBehindTruck(oncomingTruck),
    with regionContainedIn None

# Parked white SUV on the left side further ahead
parkedSpot = new OrientedPoint on visible leftCurb,
    ahead of egoSpawn by 60

parkedSUV = new Car right of parkedSpot by PARKED_SUV_OFFSET,
    with blueprint PARKED_SUV_MODEL,
    with color (1.0, 1.0, 1.0),
    with regionContainedIn None,
    with behavior StationaryBehavior()

# Ensure spatial consistency
require distance from ego to pedestrian > 20
require distance from pedestrian to parkedSUV > 10

terminate after 30 seconds