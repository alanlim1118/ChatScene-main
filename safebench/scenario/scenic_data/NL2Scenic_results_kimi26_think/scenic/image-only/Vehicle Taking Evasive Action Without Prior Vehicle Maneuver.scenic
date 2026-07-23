"""Scenario Description:

In an urban area during daylight with clear weather conditions, a vehicle is traveling straight on a road at a non-junction location with a posted speed limit of 35 mph. The vehicle, positioned in the right lane, approaches a black obstacle directly ahead in its path. To avoid a collision, the driver takes evasive action by swerving to the left, following a trajectory indicated by a dashed curved arrow that leads towards the roadside where a tree is situated. This maneuver appears to direct the vehicle across the lane of another car that is traveling straight in the adjacent left lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
param weather = "ClearNoon"
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
OBS_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(14, 16)          # ~35 mph in m/s
param OTHER_SPEED = Range(12, 15)
param OBSTACLE_DIST = Range(25, 35)
param SWERVE_TRIGGER_DIST = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior FollowLineBehavior(line, target_speed=10):
    """
    Follows the given PolylineRegion using longitudinal and latitudinal controllers.
    """
    assert line is not None
    assert isinstance(line, PolylineRegion)

    distanceToEndpoint = 5
    end_point = line[-1]

    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0

    while (distance from self to end_point) > distanceToEndpoint:
        current_speed = self.speed if self.speed is not None else 0
        cte = line.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed
        throttle = _lon_controller.run_step(speed_error)
        current_steer_angle = _lat_controller.run_step(cte)
        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle

behavior EgoBehavior(obstacle, swerveLine, target_speed=15):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to obstacle) < globalParameters.SWERVE_TRIGGER_DIST:
        do FollowLineBehavior(line=swerveLine, target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane for the ego (right lane)
egoLane = Uniform(*network.lanes)

# Spawn points
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Black obstacle directly ahead in the right lane
obstacleSpawnPt = new OrientedPoint ahead of egoSpawnPt by globalParameters.OBSTACLE_DIST

# Adjacent left lane vehicle (offset left by approx. lane width)
otherSpawnPt = new OrientedPoint left of egoSpawnPt by 3.5

# Tree situated on the left roadside, slightly ahead of the obstacle
treeBasePt = new OrientedPoint ahead of obstacleSpawnPt by 5
treeSpawnPt = new OrientedPoint left of treeBasePt by 7

# Curved swerve trajectory from right lane across left lane to roadside
p1 = new OrientedPoint ahead of egoSpawnPt by (globalParameters.OBSTACLE_DIST - 10)
p2_helper = new OrientedPoint ahead of p1 by 5
p2 = new OrientedPoint left of p2_helper by 1.5
p3_helper = new OrientedPoint ahead of p2 by 5
p3 = new OrientedPoint left of p3_helper by 3.5
p4 = new OrientedPoint at treeSpawnPt

swerveLine = PolylineRegion([p1, p2, p3, p4])

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(obstacle, swerveLine, target_speed=globalParameters.EGO_SPEED),
    with regionContainedIn None

obstacle = new Car at obstacleSpawnPt,
    with heading egoSpawnPt.heading,
    with blueprint OBS_MODEL,
    with color Color(0, 0, 0),
    with behavior WaitBehavior(),
    with regionContainedIn None

otherCar = new Car at otherSpawnPt,
    with heading egoSpawnPt.heading,
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OTHER_SPEED),
    with regionContainedIn None

tree = new Prop at treeSpawnPt,
    with blueprint "static.prop.tree",
    with regionContainedIn None

# Requirements to ensure valid geometry
require globalParameters.OBSTACLE_DIST > 15
require (distance from egoSpawnPt to obstacleSpawnPt) > 20