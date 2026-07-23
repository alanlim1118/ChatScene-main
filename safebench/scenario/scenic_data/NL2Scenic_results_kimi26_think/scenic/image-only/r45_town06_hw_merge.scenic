"""Scenario Description:

This top-down view shows a highway running vertically between a residential area with houses on the left and a dense forest on the right. The ego vehicle, a green car, travels straight in the rightmost lane, following a red lead vehicle and a yellow car further ahead. Meanwhile, a blue adversary vehicle drives along a curved on-ramp merging from the right, aiming to join the flow of traffic in the lane occupied by the other vehicles.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_SPEED = Range(8, 12)
param OPT_YELLOW_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(6, 10)

param OPT_LEAD_DISTANCE = Range(15, 25)
param OPT_YELLOW_DISTANCE = Range(35, 50)
param OPT_ADV_DISTANCE = Range(10, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior DriveForward(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a junction representing a highway on-ramp merge
junction = Uniform(*network.intersections)

# Identify the highway through-lane (straight maneuver) and the on-ramp lane
highwayManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, junction.maneuvers))
highwayLane = highwayManeuver.startLane

# The ramp is another incoming lane to the same junction
rampLane = Uniform(*filter(lambda l: l != highwayLane, junction.incomingLanes))

# Ego spawn point on the highway lane
egoSpawnPt = new OrientedPoint in highwayLane.centerline

# Lead vehicle ahead of ego in the same lane
leadSpawnPt = new OrientedPoint following highwayLane.orientation from egoSpawnPt for globalParameters.OPT_LEAD_DISTANCE

# Yellow car further ahead of ego in the same lane
yellowSpawnPt = new OrientedPoint following highwayLane.orientation from egoSpawnPt for globalParameters.OPT_YELLOW_DISTANCE

# Adversary spawn point on the on-ramp lane, some distance back from the merge
advSpawnPt = new OrientedPoint following (rampLane.orientation + 180 deg) from rampLane.centerline.end for globalParameters.OPT_ADV_DISTANCE,
    with heading rampLane.orientation

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color Color(0, 1, 0),
    with behavior DriveForward(globalParameters.OPT_EGO_SPEED)

leadVehicle = new Car at leadSpawnPt,
    with blueprint EGO_MODEL,
    with color Color(1, 0, 0),
    with behavior DriveForward(globalParameters.OPT_LEAD_SPEED)

yellowCar = new Car at yellowSpawnPt,
    with blueprint EGO_MODEL,
    with color Color(1, 1, 0),
    with behavior DriveForward(globalParameters.OPT_YELLOW_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint EGO_MODEL,
    with color Color(0, 0, 1),
    with behavior DriveForward(globalParameters.OPT_ADV_SPEED)

require 30 <= (distance to junction) <= 60
terminate when (distance from ego to junction) > 80