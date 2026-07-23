"""Scenario Description:

Under misty weather conditions, the ego vehicle travels straight through an urban, multi-lane four-way intersection. As the ego vehicle approaches the junction, it navigates past several crossing vehicles, including one turning left from the left arm, another passing straight from the left arm, a third passing straight from the opposite arm, and a fourth passing straight from the right arm. Furthermore, three pedestrians are observed crossing the intersection, with one pedestrian crossing from the far side and two pedestrians crossing from the near side relative to the ego vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
param weather = 'WetCloudyNoon'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(5, 10)

PED_SPEED = 1.0
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

behavior AdvBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

behavior PedestrianBehavior(speed):
    do WalkForwardBehavior(speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

import math

def laneHeading(lane):
    pts = lane.centerline.points
    if len(pts) < 2:
        return 0.0
    p1 = pts[-2]
    p2 = pts[-1]
    return math.atan2(p2.y - p1.y, p2.x - p1.x)

def normalizeAngle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego setup: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Determine arm headings relative to ego
egoHeading = laneHeading(egoInitLane)
expectedLeft = normalizeAngle(egoHeading - math.pi/2)
expectedRight = normalizeAngle(egoHeading + math.pi/2)
expectedOpp = normalizeAngle(egoHeading + math.pi)

# Group incoming lanes by arm
leftLanes = [lane for lane in intersection.incomingLanes 
             if abs(normalizeAngle(laneHeading(lane) - expectedLeft)) < 0.5]
rightLanes = [lane for lane in intersection.incomingLanes 
              if abs(normalizeAngle(laneHeading(lane) - expectedRight)) < 0.5]
oppLanes = [lane for lane in intersection.incomingLanes 
            if abs(normalizeAngle(laneHeading(lane) - expectedOpp)) < 0.5]

# Select lanes with required maneuvers
leftTurnLane = Uniform(*[lane for lane in leftLanes 
                         if any(m.type is ManeuverType.LEFT_TURN for m in lane.maneuvers)])
leftStraightLane = Uniform(*[lane for lane in leftLanes 
                              if any(m.type is ManeuverType.STRAIGHT for m in lane.maneuvers)])
oppStraightLane = Uniform(*[lane for lane in oppLanes 
                             if any(m.type is ManeuverType.STRAIGHT for m in lane.maneuvers)])
rightStraightLane = Uniform(*[lane for lane in rightLanes 
                               if any(m.type is ManeuverType.STRAIGHT for m in lane.maneuvers)])

# Maneuvers
leftTurnManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, leftTurnLane.maneuvers))
leftStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leftStraightLane.maneuvers))
oppStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, oppStraightLane.maneuvers))
rightStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightStraightLane.maneuvers))

# Trajectories
leftTurnTrajectory = [leftTurnLane, leftTurnManeuver.connectingLane, leftTurnManeuver.endLane]
leftStraightTrajectory = [leftStraightLane, leftStraightManeuver.connectingLane, leftStraightManeuver.endLane]
oppStraightTrajectory = [oppStraightLane, oppStraightManeuver.connectingLane, oppStraightManeuver.endLane]
rightStraightTrajectory = [rightStraightLane, rightStraightManeuver.connectingLane, rightStraightManeuver.endLane]

# Spawn points
leftTurnSpawn = new OrientedPoint in leftTurnLane.centerline
leftStraightSpawn = new OrientedPoint in leftStraightLane.centerline
oppStraightSpawn = new OrientedPoint in oppStraightLane.centerline
rightStraightSpawn = new OrientedPoint in rightStraightLane.centerline

# Pedestrian positions
oppHeading = laneHeading(oppStraightLane)

# Near side pedestrians (ego's side of intersection)
nearSidePt1 = new OrientedPoint at egoSpawnPt offset by (2, 3), facing egoHeading + math.pi/2
nearSidePt2 = new OrientedPoint at egoSpawnPt offset by (7, 3), facing egoHeading + math.pi/2

# Far side pedestrian (opposite side of intersection)
farSidePt = new OrientedPoint at oppStraightSpawn offset by (2, 3), facing oppHeading + math.pi/2

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Crossing vehicles
leftTurnCar = new Car at leftTurnSpawn,
    with blueprint MODEL,
    with behavior AdvBehavior(leftTurnTrajectory, globalParameters.ADV_SPEED)

leftStraightCar = new Car at leftStraightSpawn,
    with blueprint MODEL,
    with behavior AdvBehavior(leftStraightTrajectory, globalParameters.ADV_SPEED)

oppStraightCar = new Car at oppStraightSpawn,
    with blueprint MODEL,
    with behavior AdvBehavior(oppStraightTrajectory, globalParameters.ADV_SPEED)

rightStraightCar = new Car at rightStraightSpawn,
    with blueprint MODEL,
    with behavior AdvBehavior(rightStraightTrajectory, globalParameters.ADV_SPEED)

# Pedestrians
pedFar = new Pedestrian at farSidePt,
    with behavior PedestrianBehavior(PED_SPEED)

pedNear1 = new Pedestrian at nearSidePt1,
    with behavior PedestrianBehavior(PED_SPEED)

pedNear2 = new Pedestrian at nearSidePt2,
    with behavior PedestrianBehavior(PED_SPEED)

# Requirements
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from leftTurnCar to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from leftStraightCar to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from oppStraightCar to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from rightStraightCar to intersection) <= ADV_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST