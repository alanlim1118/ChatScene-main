"""Scenario Description:

The ego vehicle proceeds forward along a narrow, snow-covered rural road under overcast winter daylight conditions, flanked by bare trees, utility poles, and low roadside structures including a building with red walls on the left and a white prefabricated shed on the right. An oncoming light-colored passenger car approaches in the opposite lane but suddenly loses traction on the icy road surface, causing it to skid sideways and drift directly into the ego vehicle's path. This loss of control creates an imminent head-on or side-swipe collision scenario as the sliding vehicle crosses the center of the roadway, while a second vehicle remains visible further ahead in the distance. The completely snow-packed road surface and flat gray sky contribute to reduced traction and moderate visibility, highlighting the hazardous winter driving conditions captured by the dashcam.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param EGO_SPEED = Range(6, 10)
param ADV_SPEED = Range(8, 12)
param SKID_TRIGGER_DIST = Range(20, 35)
param ADV_SPAWN_DIST = Range(45, 65)
param SECOND_VEHICLE_DIST = Range(110, 150)
param LANE_WIDTH = 3.6

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior AdversarySkidBehavior(trigger_dist):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when distance from self to ego < trigger_dist:
        # Simulate loss of traction: hard steer into the ego lane with throttle
        take SetSteerAction(-1.0), SetThrottleAction(0.3)
        do WaitBehavior() for 3 seconds

#################################
# SPATIAL RELATIONS             #
#################################

# Select a rural road with at least one lane
road = Uniform(*network.roads)
egoLane = Uniform(*road.lanes)

# Ego spawn point on the selected lane
egoSpawn = new OrientedPoint in egoLane.centerline

# Oncoming adversary placed ahead in the opposite lane
advRef = new OrientedPoint ahead of egoSpawn by globalParameters.ADV_SPAWN_DIST
advSpawn = new OrientedPoint left of advRef by globalParameters.LANE_WIDTH,
    with heading (advRef.heading + 180 deg)

# Second vehicle visible further ahead in the oncoming lane
secondRef = new OrientedPoint ahead of egoSpawn by globalParameters.SECOND_VEHICLE_DIST
secondSpawn = new OrientedPoint left of secondRef by globalParameters.LANE_WIDTH,
    with heading (secondRef.heading + 180 deg)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle proceeding along the road
ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Light-colored oncoming passenger car that loses traction
adversary = new Car at advSpawn,
    with color Color(0.9, 0.9, 0.85),
    with behavior AdversarySkidBehavior(globalParameters.SKID_TRIGGER_DIST)

# Second vehicle visible in the distance
secondCar = new Car at secondSpawn,
    with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

require distance from ego to adversary > 30
terminate after 15 seconds