"""Scenario Description:

A vehicle is traveling straight in a rural area at night under clear weather conditions with a posted speed limit of 55 mph or more. The scene depicts a top-down view of the vehicle in a lane marked by dashed lines, moving directly toward an animal standing in its path ahead. This encounter takes place at a non-junction location, indicating a potential hazard where the vehicle's forward trajectory intersects with the animal's position on the roadway.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ANIMAL_MODEL = "walker.pedestrian.0001"  # Using pedestrian as animal proxy since CARLA lacks native animals

# Night time clear weather
param weather = 'ClearNight'

# Speed limit >= 55 mph ≈ 24.6 m/s; we set ego target speed accordingly
param EGO_SPEED = Range(24.6, 30.0)  # ~55-67 mph in m/s
param ANIMAL_DISTANCE_AHEAD = Range(30, 60)  # Distance to animal ahead

#################################
# AGENT BEHAVIORS               #
#################################

behavior StandStillBehavior():
    while True:
        wait

behavior DriveStraightBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=5):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-junction lane section with dashed markings and sufficient speed limit
eligibleLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        # Filter for non-junction sections
        if sec._isJunction:
            continue
        # Check for dashed line marking (left or right boundary)
        hasDashed = False
        if sec._laneToLeft is not None and sec.leftBoundary.markingType == MarkingType.DASHED:
            hasDashed = True
        if sec._laneToRight is not None and sec.rightBoundary.markingType == MarkingType.DASHED:
            hasDashed = True
        # Check speed limit >= 55 mph (24.6 m/s)
        if hasDashed and sec.speedLimit >= 24.6:
            eligibleLaneSections.append(sec)

require len(eligibleLaneSections) > 0

egoLaneSec = Uniform(*eligibleLaneSections)
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

animalSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for resample(ANIMAL_DISTANCE_AHEAD),
    facing roadDirection

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn egoLaneSec,
    facing roadDirection,
    with behavior DriveStraightBehavior(globalParameters.EGO_SPEED)

animal = new Pedestrian at animalSpawnPt,
    with blueprint ANIMAL_MODEL,
    with regionContainedIn egoLaneSec,
    facing roadDirection,
    with behavior StandStillBehavior()

# Ensure the scenario is at a non-junction location
require not egoLaneSec._isJunction

# Terminate after the ego has passed the animal or traveled far enough
terminate when distance from ego to animalSpawnPt > 80 or simulation().currentTime > 15