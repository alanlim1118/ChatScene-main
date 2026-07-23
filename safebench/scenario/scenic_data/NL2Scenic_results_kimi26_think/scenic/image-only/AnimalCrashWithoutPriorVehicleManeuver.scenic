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
# 55 mph ≈ 24.6 m/s; target speed range in m/s
param OPT_EGO_SPEED = Range(25, 30)
param OPT_ANIMAL_DISTANCE = Range(40, 80)

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    while True:
        wait

behavior EgoDriveBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight, forward lane on a two-lane rural road (dashed center line)
# indicated by the presence of a forward lane to the left.
candidateLaneSecs = []
for lane in network.lanes:
    if lane.isForward:
        for laneSec in lane.sections:
            if laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
                candidateLaneSecs.append(laneSec)

egoLaneSec = Uniform(*candidateLaneSecs)
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle traveling straight at night
ego = new Car at egoSpawnPt,
    facing roadDirection,
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior(globalParameters.OPT_EGO_SPEED)

# Animal standing stationary in the path of the ego vehicle
AnimalAgent = new Pedestrian following roadDirection from ego for globalParameters.OPT_ANIMAL_DISTANCE,
    with regionContainedIn egoLaneSec,
    with behavior StationaryBehavior()

# Ensure clear night weather
param weather = 'ClearNight'

# Non-junction requirement: ensure the selected lane section is valid
require egoLaneSec is not None