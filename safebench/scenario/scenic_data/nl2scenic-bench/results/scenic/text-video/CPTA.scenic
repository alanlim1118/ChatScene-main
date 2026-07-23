"""Scenario Description:

In a top-down simulated environment, a white rectangular ego vehicle travels from right to left approaching a four-way intersection. As the vehicle enters the junction, it initiates a right turn onto the perpendicular road. An adult pedestrian is visible crossing the path of the vehicle, walking across the road segment into which the ego vehicle is turning. The ego vehicle maintains its trajectory and speed without applying any braking action. The scenario depicts the vehicle's frontal structure striking the pedestrian as the turn is executed.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(1, 4)
param OPT_ADV_DISTANCE = Range(10, 18)
param OPT_PARAM_LANE_WIDTH = 6

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoNoBrakeBehavior():
    """Ego follows trajectory at constant speed without braking."""
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

behavior PedestrianCrossingBehavior(actor_reference, adv_speed, adv_distance):
    """Pedestrian crosses the target lane when ego is within trigger distance."""
    while distance from self to actor_reference > adv_distance:
        wait
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(adv_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select a right-turn maneuver at this intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

# Ego spawn point on the start lane centerline (approaching from right to left)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Pedestrian spawn point: on the end lane (the road ego turns into), positioned
# so that the pedestrian crosses perpendicular to that road
endLaneCenterPt = new OrientedPoint in egoManeuver.endLane.centerline
pedSpawnPt = new OrientedPoint at endLaneCenterPt,
    with heading egoManeuver.endLane.centerline.heading + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color "255,255,255",
    with behavior EgoNoBrakeBehavior()

AdvAgent = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE)

# Ensure ego starts at a reasonable distance from the intersection
require 30 <= (distance from ego to intersection) <= 55

# Ensure pedestrian is placed within the end lane region for realistic crossing
require distance from AdvAgent to egoManeuver.endLane.centerline < OPT_PARAM_LANE_WIDTH

terminate after 30 seconds