"""Scenario Description:

The leading vehicle decelerates suddenly due to an obstacle and the ego-vehicle must perform an emergency brake or an avoidance maneuver.

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

LEAD_CAR_SPEED = 10
LEADCAR_BRAKING_THRESHOLD = 10

BRAKE_ACTION = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

# LEAD CAR BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior LeadingCarBehavior(speed=10):
    try:
        do FollowLaneBehavior(speed)

    interrupt when withinDistanceToAnyObjs(self, LEADCAR_BRAKING_THRESHOLD):
        take SetBrakeAction(BRAKE_ACTION)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
        with blueprint EGO_MODEL

leadCar = new Car following roadDirection from egoSpawnPt for Range(10, 15),
        with behavior LeadingCarBehavior(LEAD_CAR_SPEED)

obstacle = new Trash following roadDirection from leadCar for Range(30, 50)

require (distance to intersection) > 80
terminate when ego.speed < 0.1 and (distance to obstacle) < 30
