"""Scenario Description:

From a top-down perspective of a sunny urban intersection surrounded by high-rise buildings, a blue ego vehicle travels straight in the left lane behind a yellow truck. As the vehicles approach the intersection, the yellow truck ahead suddenly brakes and comes to a halt just past the crosswalk. The blue ego vehicle subsequently slows down and stops behind the truck. Meanwhile, in the adjacent right lane, a grey sedan and a red sedan continue driving straight through the intersection, passing the stationary vehicles in the left lane.

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
TRUCK_MODEL = "vehicle.carlamotors.carlacola"
SEDAN_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(8, 12)
param TRUCK_SPEED = Range(8, 12)
param SEDAN_SPEED = Range(8, 12)

TRUCK_BRAKE_DIST_FROM_CROSSWALK = Range(2, 5)
EGO_STOP_DIST_BEHIND_TRUCK = Range(3, 6)
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior TruckBrakeBehavior(brake_point):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)
    interrupt when (distance from self to brake_point) <= TRUCK_BRAKE_DIST_FROM_CROSSWALK:
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        while True:
            wait

behavior EgoFollowAndStopBehavior(lead_vehicle):
    try:
        do FollowLeadingVehicleBehavior(target_speed=globalParameters.EGO_SPEED, lead_vehicle=lead_vehicle)
    interrupt when (distance from self to lead_vehicle) <= EGO_STOP_DIST_BEHIND_TRUCK and lead_vehicle.speed < 0.5:
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        while True:
            wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Left lane maneuver for ego and truck
leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
leftInitLane = leftManeuver.startLane
leftTrajectory = [leftInitLane, leftManeuver.connectingLane, leftManeuver.endLane]

# Find the right lane adjacent to the left lane
rightInitLane = Uniform(*filter(lambda l: l is not leftInitLane and 
    (l.leftNeighbor is leftInitLane or l.rightNeighbor is leftInitLane),
    leftInitLane.parentRoad.lanes))
rightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightInitLane.maneuvers))
rightTrajectory = [rightInitLane, rightManeuver.connectingLane, rightManeuver.endLane]

# Spawn points along left lane centerline
truckSpawnPt = new OrientedPoint in leftInitLane.centerline
egoSpawnPt = new OrientedPoint behind truckSpawnPt by Range(10, 15)

# Brake point: just past the crosswalk on the connecting/end lane
crosswalkRegion = leftManeuver.connectingLane.crosswalks[0] if leftManeuver.connectingLane.crosswalks else leftManeuver.endLane
brakePoint = new OrientedPoint at crosswalkRegion.center, with heading leftInitLane.centerline.heading

# Right lane spawn points
greySedanSpawnPt = new OrientedPoint in rightInitLane.centerline
redSedanSpawnPt = new OrientedPoint behind greySedanSpawnPt by Range(8, 12)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "Blue",
    with behavior EgoFollowAndStopBehavior(truck)

truck = new Car at truckSpawnPt,
    with blueprint TRUCK_MODEL,
    with color "Yellow",
    with behavior TruckBrakeBehavior(brakePoint)

greySedan = new Car at greySedanSpawnPt,
    with blueprint SEDAN_MODEL,
    with color "Grey",
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.SEDAN_SPEED, trajectory=rightTrajectory)

redSedan = new Car at redSedanSpawnPt,
    with blueprint SEDAN_MODEL,
    with color "Red",
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.SEDAN_SPEED, trajectory=rightTrajectory)

require 30 <= (distance from truckSpawnPt to intersection) <= 50
require monitor TrafficLights()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        setClosestTrafficLightStatus(ego, "green")
        setClosestTrafficLightStatus(truck, "green")
        setClosestTrafficLightStatus(greySedan, "green")
        setClosestTrafficLightStatus(redSedan, "green")
        wait

terminate when (distance from ego to egoSpawnPt) > TERM_DIST