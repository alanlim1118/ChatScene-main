"""Scenario Description:

Figure 33 presents a top-down visualization of a "Move Out of Travel Lane/Park Test Scenario" on a straight urban street divided by a dashed white line. An autonomous driving system (ADS) equipped vehicle, highlighted with a green outline, is traveling in the left lane and needs to exit the active travel lane. To its right, two stationary white vehicles are parked in a line, creating a gap labeled as the "Desired Parking Location." A curved arrow illustrates the intended path of the green vehicle as it maneuvers from the travel lane into this specific parking spot between the stationary cars, fulfilling the objective of moving out of traffic to allow for passenger embarkation or disembarkation.

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
PARKED_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(2, 5)
param OPT_PARK_DIST = Range(20, 40)
param OPT_PARK_LATERAL = Range(3.5, 5.0)
param OPT_GAP_SIZE = Range(6, 8)
param OPT_BRAKE_DIST = Range(5, 8)

CAR_LENGTH = 4.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(park_target, speed, stop_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (distance from self to park_target <= stop_dist):
        take SetBrakeAction(1)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline

# Desired parking location on the right side of the road
parkBasePt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_PARK_DIST
parkSpotPt = new OrientedPoint right of parkBasePt by globalParameters.OPT_PARK_LATERAL

# Two stationary vehicles flanking the parking spot
rearCarPt = new OrientedPoint following egoInitLane.orientation from parkSpotPt for -(globalParameters.OPT_GAP_SIZE/2 + CAR_LENGTH/2)
frontCarPt = new OrientedPoint following egoInitLane.orientation from parkSpotPt for (globalParameters.OPT_GAP_SIZE/2 + CAR_LENGTH/2)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(parkSpotPt, globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

parkedCar1 = new Car at rearCarPt,
    with heading egoInitLane.orientation,
    with blueprint PARKED_MODEL,
    with behavior WaitBehavior()

parkedCar2 = new Car at frontCarPt,
    with heading egoInitLane.orientation,
    with blueprint PARKED_MODEL,
    with behavior WaitBehavior()

require 40 <= (distance to intersection) <= 60
terminate when distance from ego to intersection > 60