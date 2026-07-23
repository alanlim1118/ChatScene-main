"""Scenario Description:

The ego car travels straight forward within its lane. Ahead of it, an adversarial object exits parallel forward, leaving the ego-traffic area to the right.

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

param OPT_ADV_SPEED = Range(3, 5)
param OPT_ADV_START_DIST = Range(15, 25)       # Adversary starts ahead of ego
param OPT_ADV_EXIT_DIST = Range(8, 15)         # Distance at which adversary begins exiting right

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversarialExitRightBehavior(target_speed=5, exit_distance=10):
    """
    Drives forward in lane initially, then steers to the right edge to exit
    the ego's traffic area.
    """
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0

    # Phase 1: Drive straight in lane center
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego) <= exit_distance

    # Phase 2: Exit to the right by following right edge of current lane
    while True:
        if self.lane is None or not hasattr(self.lane, 'rightEdge') or self.lane.rightEdge is None:
            break

        trajectory_edge = self.lane.rightEdge

        current_speed = self.speed if self.speed is not None else 0
        speed_error = target_speed - current_speed
        throttle = _lon_controller.run_step(speed_error)

        cte = trajectory_edge.signedDistanceTo(self.position)
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

# Place adversary ahead of ego in the same lane
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior AdversarialExitRightBehavior(
        target_speed=globalParameters.OPT_ADV_SPEED,
        exit_distance=globalParameters.OPT_ADV_EXIT_DIST
    )

require distance to intersection >= 80
terminate when (distance from ego to AdvAgent > 50) or (simulation().currentTime > 30)
