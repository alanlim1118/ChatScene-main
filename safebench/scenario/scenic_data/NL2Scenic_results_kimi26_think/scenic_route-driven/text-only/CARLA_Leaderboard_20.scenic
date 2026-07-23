"""Scenario Description:

The ego-vehicle encounters a pedestrian emerging from behind a parked vehicle and advancing into the lane. The ego-vehicle must brake or maneuver to avoid it.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_PED_SPEED = Range(1.0, 2.5)
param OPT_TRIGGER_DISTANCE = Range(12, 20)
param OPT_PARKED_OFFSET = Range(0.5, 1.5)
param OPT_PED_BEHIND_OFFSET = Range(0.5, 2.0)

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedestrianEmergenceBehavior():
    while distance from self to ego > globalParameters.OPT_TRIGGER_DISTANCE:
        wait
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(globalParameters.OPT_PED_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt, with blueprint EGO_MODEL

# Parked car on the right side of the road
rightCurb = ego.laneGroup.curb
parkedSpot = new OrientedPoint on visible rightCurb

parkedCar = new Car right of parkedSpot by globalParameters.OPT_PARKED_OFFSET,
    with heading parkedSpot.heading,
    with regionContainedIn None

# Pedestrian emerges from behind the parked car and advances into the lane
new Pedestrian behind parkedCar by globalParameters.OPT_PED_BEHIND_OFFSET,
    facing 90 deg relative to parkedCar,
    with regionContainedIn None,
    with behavior PedestrianEmergenceBehavior()

require distance to intersection > 50
terminate after 30 seconds
