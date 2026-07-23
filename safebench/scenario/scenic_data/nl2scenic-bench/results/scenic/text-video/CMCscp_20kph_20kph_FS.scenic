"""Scenario Description:

In a simulated top-down view of a four-way intersection, a white ego vehicle travels horizontally from left to right along the upper lane of the main road, maintaining a steady speed. Parallel to it in the lower lane, a long red vehicle moves in the same direction. A motorcyclist, visible as a small black object, enters the junction from the bottom vertical road and proceeds upwards on a perpendicular path, crossing directly into the ego vehicle's trajectory. The ego vehicle continues straight without braking or steering away, resulting in a collision where the frontal structure of the white vehicle strikes the front of the motorcyclist as they converge in the center of the intersection.

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
RED_VEHICLE_MODEL = "vehicle.tesla.model3"
MOTORCYCLE_MODEL = "vehicle.yamaha.yzf"

param EGO_SPEED = Range(8, 12)
param RED_VEHICLE_SPEED = Range(8, 12)
param MOTO_SPEED = Range(6, 10)

EGO_COLOR = (255, 255, 255)       # White
RED_COLOR = (200, 30, 30)         # Red
MOTO_COLOR = (20, 20, 20)        # Black

TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantSpeedStraightBehavior(speed, trajectory):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Spawn ego in the upper lane (left-to-right)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Red vehicle in the lane to the right of ego (lower lane, same direction)
require network.laneSectionAt(egoSpawnPt) is not None
require network.laneSectionAt(egoSpawnPt).laneToRight is not None
redLane = network.laneSectionAt(egoSpawnPt).laneToRight.lane
redSpawnOffset = Range(-5, 5)  # Slight longitudinal offset relative to ego
redProjectPt = redLane.centerline.project(egoSpawnPt.position)
redSpawnPt = new OrientedPoint at redProjectPt,
    with heading redLane.orientation[redProjectPt]
redSpawnPt = new OrientedPoint ahead of redSpawnPt by redSpawnOffset

# Motorcyclist enters from the bottom vertical road going up (perpendicular to ego)
# Find the incoming lane from the south that goes straight through the intersection
motoManeuverCandidates = filter(
    lambda m: m.type is ManeuverType.STRAIGHT and abs(m.startLane.centerline.end.heading - egoInitLane.centerline.end.heading) > 45 deg,
    intersection.maneuvers
)
motoManeuver = Uniform(*motoManeuverCandidates)
motoInitLane = motoManeuver.startLane
motoTrajectory = [motoInitLane, motoManeuver.connectingLane, motoManeuver.endLane]

# Spawn motorcycle on its incoming lane, positioned so it arrives at intersection center
# roughly when ego does
motoSpawnPt = new OrientedPoint in motoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior ConstantSpeedStraightBehavior(globalParameters.EGO_SPEED, egoTrajectory)

redVehicle = new Car at redSpawnPt,
    with blueprint RED_VEHICLE_MODEL,
    with color RED_COLOR,
    with behavior ConstantSpeedStraightBehavior(globalParameters.RED_VEHICLE_SPEED, 
        [redLane, egoManeuver.connectingLane._parallelLanes.get(redLane, egoManeuver.connectingLane), 
         egoManeuver.endLane._parallelLanes.get(redLane, egoManeuver.endLane)])

motorcycle = new Motorcycle at motoSpawnPt,
    with blueprint MOTORCYCLE_MODEL,
    with color MOTO_COLOR,
    with behavior ConstantSpeedStraightBehavior(globalParameters.MOTO_SPEED, motoTrajectory)

# Ensure agents start at reasonable distances from the intersection
require 30 <= (distance from ego to intersection) <= 50
require 30 <= (distance from motorcycle to intersection) <= 50

terminate when (distance from ego to egoSpawnPt) > TERM_DIST