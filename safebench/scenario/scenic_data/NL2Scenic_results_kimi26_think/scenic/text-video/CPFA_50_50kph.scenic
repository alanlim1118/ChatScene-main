"""Scenario Description:

The ego vehicle is driving on a straight road at a steady speed of 50 km/h. A pedestrian enters from the left side (farside) of the road and runs perpendicularly across the vehicle's path. The ego vehicle continues its forward motion without applying any brakes, maintaining a direct course toward the crossing pedestrian. As the sequence progresses, the pedestrian moves closer to the center of the lane, placing themselves directly in the path of the vehicle's front end, culminating in a collision where the vehicle strikes the pedestrian.

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
EGO_SPEED = 50 / 3.6             # 50 km/h in m/s

param OPT_PED_SPEED = Range(2.0, 3.5)           # Pedestrian running speed (m/s)
param OPT_CROSSING_DISTANCE = Range(15, 25)     # Distance ahead to crossing point (m)
param OPT_LATERAL_OFFSET = Range(2.5, 4.0)      # Lateral distance from lane center to pedestrian start (m)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    # Maintain constant forward speed; do not brake
    do FollowLaneBehavior(speed)

behavior RunAcross(speed):
    # Run continuously in the direction of the current heading
    take SetWalkingDirectionAction(self.heading)
    take SetWalkingSpeedAction(speed)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
crossPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_CROSSING_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

# Pedestrian on the farside (left) of the road, running perpendicularly across the path
pedestrian = new Pedestrian left of crossPt by globalParameters.OPT_LATERAL_OFFSET,
    with heading crossPt.heading - 90 deg,
    with regionContainedIn None,
    with behavior RunAcross(globalParameters.OPT_PED_SPEED)

require 40 <= (distance to intersection) <= 60

terminate after 20 seconds