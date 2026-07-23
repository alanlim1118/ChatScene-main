"""Scenario Description:

The ego car drives straight forward within its lane. Meanwhile, an adversarial motorcycle passes it from the adjacent left lane.

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
ADV_MODEL = "vehicle.kawasaki.ninja"

param OPT_EGO_SPEED = Range(3, 5)
param OPT_ADV_SPEED = Range(6, 9)  # Motorcycle must be faster to pass
param OPT_ADV_START_DIST = Range(15, 25)  # Start behind ego so it can overtake

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdversarialPassingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a valid left lane for passing
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward):
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial motorcycle in the left lane, behind the ego vehicle
adjLaneSec = egoLaneSec._laneToLeft
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for -globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Motorcycle at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with blueprint ADV_MODEL,
    with behavior AdversarialPassingBehavior()

require distance from egoSpawnPt to intersection >= 80  # Keep away from intersections

terminate when distance from ego to AdvAgent > 60  # End after motorcycle has passed