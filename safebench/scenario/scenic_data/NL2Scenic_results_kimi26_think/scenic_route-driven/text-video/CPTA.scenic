"""Scenario Description:

A white rectangular ego vehicle travels from right to left approaching a four-way intersection. As the vehicle enters the junction, it initiates a right turn onto the perpendicular road. An adult pedestrian is visible crossing the path of the vehicle, walking across the road segment into which the ego vehicle is turning. The ego vehicle maintains its trajectory and speed without applying any braking action. The scenario depicts the vehicle's frontal structure striking the pedestrian as the turn is executed.

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

param OPT_PED_SPEED = Range(1, 3)

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedestrianWalkBehavior(speed):
    take SetWalkingSpeedAction(speed)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Place pedestrian on the end lane (the road segment into which ego turns),
# a short distance from the intersection, facing across the lane.
pedRefPt = new OrientedPoint at egoManeuver.endLane.centerline.start
pedSpawnPt = new OrientedPoint ahead of pedRefPt by Range(2, 6)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color Color(1, 1, 1)

AdvAgent = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading + 90 deg,
    with regionContainedIn None,
    with behavior PedestrianWalkBehavior(globalParameters.OPT_PED_SPEED)

require 30 <= (distance to intersection) <= 50

terminate after 15 seconds
