"""Scenario Description:

While performing a maneuver, the ego-vehicle encounters an obstacle in the road, either a pedestrian or a bicycle, and must perform an emergency brake or an avoidance maneuver.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_OBSTACLE_DISTANCE = Range(30, 50)
param OPT_BRAKE_THRESHOLD = Range(12, 18)
param OPT_AVOIDANCE_THRESHOLD = Range(18, 25)

BRAKE_ACTION = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedestrianBehavior():
    # Pedestrian walks across the road perpendicular to traffic
    take SetWalkingSpeedAction(1.5)

behavior BicycleBehavior():
    # Bicycle moves slowly along the lane or crosses
    do FollowLaneBehavior(target_speed=3.0)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_AVOIDANCE_THRESHOLD):
        # Attempt avoidance if there is enough distance and an adjacent lane exists
        if self.laneSection._laneToLeft is not None and (distance from self to Obstacle > globalParameters.OPT_BRAKE_THRESHOLD):
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=globalParameters.OPT_EGO_SPEED)
        else:
            # Emergency brake as fallback or if too close for safe lane change
            take SetThrottleAction(0)
            take SetBrakeAction(BRAKE_ACTION)
            terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane that has a left neighbor for potential avoidance
laneSecsWithLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            laneSecsWithLeft.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_OBSTACLE_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Randomly choose obstacle type: pedestrian or bicycle
obstacleType = Uniform('pedestrian', 'bicycle')

if obstacleType == 'pedestrian':
    Obstacle = new Pedestrian at obstacleSpawnPt offset by Range(-2, 2) @ 0,
        with heading obstacleSpawnPt.heading + 90 deg,
        with behavior PedestrianBehavior()
else:
    Obstacle = new Bicycle at obstacleSpawnPt,
        with heading obstacleSpawnPt.heading,
        with regionContainedIn egoLaneSec,
        with behavior BicycleBehavior()

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn egoLaneSec,
    with behavior EgoBehavior()

require (distance to intersection) > 60
terminate when ego.speed < 0.1 and (distance to Obstacle) < 10