"""Scenario Description:
The ego vehicle travels in a straight line for at least two seconds before 
encountering a slower moving vehicle target directly ahead in the same lane 
with an offset of less than 0.5 meters while maintaining a constant speed 
difference of 50 km/h between the two vehicles.
"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Vehicle model
CAR_MODEL = 'vehicle.tesla.model3'

# Speed definitions (m/s)
# 50 km/h is approximately 13.89 m/s
EGO_SPEED = 30  # ~108 km/h
SPEED_DIFF_MS = 50 / 3.6
TARGET_SPEED = EGO_SPEED - SPEED_DIFF_MS

# Initial distance calculation:
# To ensure the ego travels for at least 2 seconds before the gap closes significantly,
# we set a distance greater than (Relative Speed * 2 seconds).
# 13.89 * 2 = 27.78 meters. We use a range above this.
INIT_DIST = Range(45, 60)

# Lateral offset < 0.5 meters
LATERAL_OFFSET = Range(-0.45, 0.45)

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantDrive(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for long highway lanes to ensure a straight trajectory
straightLanes = [l for l in network.lanes if l.centerline.length > 500]
# Exclude lanes that are part of an intersection to maintain a straight line
highwayLanes = [l for l in straightLanes if not any(isinstance(r, Intersection) for r in l.roads)]

# Ensure we have suitable lanes
assert len(highwayLanes) > 0, "No suitable long straight lanes found in Town06."

initLane = Uniform(*highwayLanes)
spawnPt = new OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego Vehicle
ego = new Car at spawnPt,
    with rolename 'hero',
    with blueprint CAR_MODEL,
    with behavior ConstantDrive(EGO_SPEED)

# Spawn Target Vehicle directly ahead in the same lane
# The 'offset by' specifier uses a vector where x is lateral and y is longitudinal
target = new Car following roadDirection from ego for INIT_DIST,
    offset by (LATERAL_OFFSET @ 0),
    with blueprint CAR_MODEL,
    with behavior ConstantDrive(TARGET_SPEED)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure vehicles are far from intersections to keep the "straight line" requirement
require (distance from ego to intersection) > 100
require (distance from target to intersection) > 100

# Ensure they remain in the same lane
require target.lane is ego.lane

# Terminate after the ego has traveled a significant distance
terminate when (distance to spawnPt) > 250