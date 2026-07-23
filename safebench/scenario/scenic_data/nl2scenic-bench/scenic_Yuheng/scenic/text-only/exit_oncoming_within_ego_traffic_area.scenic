"""Scenario Description:

The ego car travels straight forward within its lane. Ahead of it, an oncoming adversarial object exits the ego's traffic area by veering into the adjacent lane.

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

param OPT_EGO_SPEED = Range(4, 6)
param OPT_ADV_SPEED = Range(3, 5)
param OPT_ADV_START_DIST = Range(30, 50)       # Distance ahead of ego where adv spawns (oncoming)
param OPT_ADV_VEER_DIST = Range(15, 25)        # Distance from ego at which adv starts veering
param OPT_EGO_BRAKE_DIST = Range(8, 12)        # Distance at which ego brakes if adv is still close

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_EGO_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior OncomingVeerBehavior(target_speed=5, veer_trigger_dist=20):
    """
    Drives oncoming toward ego, then veers laterally out of ego's lane
    into the adjacent lane once within veer_trigger_dist.
    """
    past_steer_angle = 0
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)

    while True:
        dist_to_ego = distance from self to ego
        current_speed = self.speed if self.speed is not None else 0
        speed_error = target_speed - current_speed
        throttle = _lon_controller.run_step(speed_error)

        if dist_to_ego > veer_trigger_dist:
            # Still approaching: follow current lane centerline normally
            cte = self.lane.centerline.signedDistanceTo(self.position)
            steer = _lat_controller.run_step(cte)
        else:
            # Veer phase: target the right edge of current lane (exiting toward adjacent lane)
            if hasattr(self.lane, 'rightEdge') and self.lane.rightEdge is not None:
                cte = self.lane.rightEdge.signedDistanceTo(self.position)
            elif hasattr(self.lane, 'leftEdge') and self.lane.leftEdge is not None:
                cte = self.lane.leftEdge.signedDistanceTo(self.position)
            else:
                cte = 0
            steer = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, steer, past_steer_angle)
        past_steer_angle = steer

        # Terminate after passing ego or going too far
        if dist_to_ego < 2:
            break

#################################
# SPATIAL RELATIONS             #
#################################

# Find forward lane sections that have an adjacent lane in the same direction
validLaneSecs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and (sec._laneToLeft is not None or sec._laneToRight is not None):
            validLaneSecs.append(sec)

egoLaneSec = Uniform(*validLaneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial agent ahead on the same lane but facing opposite direction (oncoming)
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_START_DIST,
    with heading egoSpawnPt.heading + 180 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior OncomingVeerBehavior(
        target_speed=globalParameters.OPT_ADV_SPEED,
        veer_trigger_dist=globalParameters.OPT_ADV_VEER_DIST
    )

require distance from egoSpawnPt to nearest intersection >= 80
terminate when (distance from ego to AdvAgent > 60) or (simulation().currentTime > 30)