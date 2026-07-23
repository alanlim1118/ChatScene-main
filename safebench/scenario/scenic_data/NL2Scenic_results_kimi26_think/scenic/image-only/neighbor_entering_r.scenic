"""Scenario Description:

A multi-lane roadway with traffic flowing from left to right. The ego vehicle (blue) travels straight ahead in the upper lane. A pink vehicle in the lower lane performs a lane change maneuver from the bottom lane into the central lane, entering the traffic flow from the right-hand side relative to the ego vehicle's position.

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

param EGO_SPEED = Range(8, 12)
param PINK_SPEED = Range(8, 12)
param PINK_OFFSET = Range(-10, 20)

#################################
# SPATIAL RELATIONS             #
 #################################

# Identify three-lane sections where the middle lane has both left and right forward lanes
laneSecsWithLeftAndRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftAndRightLane.append(laneSec)

middleLaneSec = resample(Uniform(*laneSecsWithLeftAndRightLane))
upperLaneSec = middleLaneSec._laneToLeft
lowerLaneSec = middleLaneSec._laneToRight

# Reference point on the middle lane centerline to align the vehicles longitudinally
refPt = new OrientedPoint on middleLaneSec.centerline

# Ego spawn point in the upper lane
egoSpawnPt = upperLaneSec.centerline.project(refPt.position)

# Pink vehicle spawn point in the lower lane, with longitudinal offset relative to ego
pinkBasePt = lowerLaneSec.centerline.project(refPt.position)
pinkSpawnPt = follow roadDirection from pinkBasePt for resample(PINK_OFFSET)

#################################
# AGENT BEHAVIORS               #
 #################################

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior PinkMergeBehavior(target_lane, target_speed):
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, is_oppositeTraffic=False, target_speed=target_speed)

#################################
# SCENARIO SPECIFICATION        #
 #################################

# Ego vehicle in the upper lane traveling straight
ego = new Car at egoSpawnPt,
    facing roadDirection,
    with regionContainedIn upperLaneSec,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# Pink vehicle in the lower lane, merging into the central lane
pinkVehicle = new Car at pinkSpawnPt,
    facing roadDirection,
    with regionContainedIn lowerLaneSec,
    with behavior PinkMergeBehavior(middleLaneSec, globalParameters.PINK_SPEED)

require laneSecsWithLeftAndRightLane is not None