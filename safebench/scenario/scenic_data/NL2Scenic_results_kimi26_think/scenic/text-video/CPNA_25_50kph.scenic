"""Scenario Description:

The ego vehicle is traveling straight on a road at a steady speed of 50 km/h. From the left side, representing the nearside, an adult pedestrian enters the scene and proceeds to walk across the vehicle's path. The ego vehicle maintains its course and speed without applying brakes, leading to a collision where the front of the vehicle impacts the pedestrian crossing the road.

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

# Ego speed fixed at 50 km/h (~13.89 m/s)
param OPT_EGO_SPEED = 13.89

# Pedestrian parameters
param OPT_PED_SPEED = Range(1.0, 1.5)
param OPT_PED_DISTANCE = Range(25, 45)
param OPT_PED_OFFSET = Range(2.0, 4.0)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior PedestrianCrossBehavior(ped_speed):
    take SetWalkingDirectionAction(self.heading)
    take SetWalkingSpeedAction(ped_speed)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
IntSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_PED_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

pedestrian = new Pedestrian left of IntSpawnPt by globalParameters.OPT_PED_OFFSET,
    with heading IntSpawnPt.heading - 90 deg,
    with regionContainedIn None,
    with behavior PedestrianCrossBehavior(globalParameters.OPT_PED_SPEED)

require 40 <= (distance to intersection) <= 60
terminate after 20 seconds