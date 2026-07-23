"""Scenario Description:

From a top-down perspective of a sunny urban intersection surrounded by high-rise buildings, a blue ego vehicle travels straight in the left lane behind a yellow truck. As the vehicles approach the intersection, the yellow truck ahead suddenly brakes and comes to a halt just past the crosswalk. The blue ego vehicle subsequently slows down and stops behind the truck. Meanwhile, in the adjacent right lane, a grey sedan and a red sedan continue driving straight through the intersection, passing the stationary vehicles in the left lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
TRUCK_MODEL = "vehicle.carlamotors.carlacola"
SEDAN_GREY_MODEL = "vehicle.audi.a2"
SEDAN_RED_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(6, 10)
param TRUCK_SPEED = Range(6, 10)
param SEDAN_SPEED = Range(8, 12)
STOPPING_DIST = 12

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior TruckBehavior(trajectory, stop_ref):
    do FollowTrajectoryBehavior(target_speed=globalParameters.TRUCK_SPEED, trajectory=trajectory) until (distance from self to stop_ref) <= 2
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

behavior EgoBehavior(trajectory, target):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory) until (distance from self to target) <= STOPPING_DIST
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select a straight maneuver in a lane that has an adjacent right lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane.laneToRight is not None, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adjacent right lane and its straight maneuver
rightLane = egoInitLane.laneToRight
rightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightLane.maneuvers))
rightTrajectory = [rightLane, rightManeuver.connectingLane, rightManeuver.endLane]

# Spawn points
truckSpawnPt = new OrientedPoint in egoInitLane.centerline
egoSpawnPt = new OrientedPoint behind truckSpawnPt by Range(8, 12)

greySpawnPt = new OrientedPoint in rightLane.centerline
redSpawnPt = new OrientedPoint behind greySpawnPt by Range(8, 12)

# Stopping point just past the crosswalk (slightly inside the intersection along the connecting lane)
stopPoint = new OrientedPoint ahead of egoManeuver.connectingLane.centerline.start by 3

#################################
# SCENARIO SPECIFICATION        #
#################################

# Yellow truck ahead in the left lane
truck = new Car at truckSpawnPt,
    with blueprint TRUCK_MODEL,
    with color (1, 1, 0),
    with behavior TruckBehavior(egoTrajectory, stopPoint)

# Blue ego behind the truck in the left lane
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory, truck)

# Grey sedan in the adjacent right lane
greySedan = new Car at greySpawnPt,
    with blueprint SEDAN_GREY_MODEL,
    with color (0.5, 0.5, 0.5),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.SEDAN_SPEED, trajectory=rightTrajectory)

# Red sedan behind the grey sedan in the right lane
redSedan = new Car at redSpawnPt,
    with blueprint SEDAN_RED_MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.SEDAN_SPEED, trajectory=rightTrajectory)

# Requirements to position the vehicles approaching the intersection
require 25 <= (distance from truckSpawnPt to intersection) <= 40
require 25 <= (distance from greySpawnPt to intersection) <= 40