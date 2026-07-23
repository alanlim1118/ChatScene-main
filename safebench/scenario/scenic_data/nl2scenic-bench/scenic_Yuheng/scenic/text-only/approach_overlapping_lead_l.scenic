"""Scenario Description:

The ego car travels straight forward within its lane. It approaches a leading adversarial object that is overlapping its lane to the left.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(2, 4)
param OPT_ADV_LEAD_DIST = Range(15, 25)       # Distance ahead of ego where adv is placed
param OPT_ADV_LEFT_OFFSET = Range(0.8, 1.5)   # Lateral offset into ego lane from left edge
param OPT_BRAKE_DIST = Range(8, 12)           # Distance at which ego brakes

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
    terminate

behavior AdversarialOverlapBehavior(target_speed=0, offset=0):
    """
    Drives along the left edge of the current lane with a lateral offset
    to partially overlap into the right (ego) lane.
    """
    past_steer_angle = 0
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)

    while True:
        current_speed = self.speed if self.speed is not None else 0
        speed_error = target_speed - current_speed
        throttle = _lon_controller.run_step(speed_error)

        if self.lane is not None and hasattr(self.lane, 'leftEdge') and self.lane.leftEdge is not None:
            left_edge = self.lane.leftEdge
            # Signed distance: positive means to the left of the edge.
            # We want to be offset to the RIGHT of the left edge (into the lane),
            # so we use negative offset as the target CTE.
            cte = left_edge.signedDistanceTo(self.position) - (-offset)
            current_steer_angle = _lat_controller.run_step(cte)
        else:
            current_steer_angle = 0

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a left neighbor (so the left edge exists and overlaps rightward)
laneSecsWithLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeft.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial agent ahead of ego on the same lane centerline
advCenterPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_LEAD_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advCenterPt,
    with heading advCenterPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior AdversarialOverlapBehavior(
        target_speed=globalParameters.OPT_ADV_SPEED,
        offset=globalParameters.OPT_ADV_LEFT_OFFSET
    )

require distance from egoSpawnPt to intersection >= 80
terminate when distance from ego to AdvAgent > 60