"""Scenario Description:

The ego vehicle (a green autonomous vehicle) is driving on a straight, undivided road in the right lane, approaching a school bus stopped in the opposing left lane facing the opposite direction with red stop arms extended (activated lights and signs to allow students to disembark).

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
BUS_MODEL = "vehicle.carlamotors.carlacola"  # Closest bus-like vehicle in default CARLA

param OPT_BUS_DISTANCE = Range(25, 45)
param OPT_LANE_WIDTH = 3.6

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Opposing lane is to the left of the ego by one lane width, facing the opposite direction
busRefPt = new OrientedPoint ahead of egoSpawnPt by globalParameters.OPT_BUS_DISTANCE
busSpawnPt = new OrientedPoint left of busRefPt by globalParameters.OPT_LANE_WIDTH,
    with heading egoSpawnPt.heading + 180 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

school_bus = new Car at busSpawnPt,
    with heading busSpawnPt.heading,
    with regionContainedIn None,
    with blueprint BUS_MODEL,
    with behavior WaitBehavior()

require 50 <= (distance to intersection) <= 80
