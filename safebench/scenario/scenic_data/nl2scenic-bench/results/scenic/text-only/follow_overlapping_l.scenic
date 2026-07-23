"""Scenario Description:

The ego car travels straight forward within its lane. It follows an adversarial object driving ahead that is overlapping its lane to the left.

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
param OPT_FOLLOW_DIST = Range(8, 15)       # Distance at which ego starts following/braking
param OPT_ADV_AHEAD_DIST = Range(15, 25)   # Initial distance of adv ahead of ego
param OPT_LANE_OVERLAP_OFFSET = Range(0.3, 0.8)  # Fraction of lane width the adv overlaps into ego's lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_FOLLOW_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior AdversarialOverlapBehavior(target_speed=5, overlap_fraction=0.5):
    """
    Drives forward while positioned to overlap into the right neighbor lane
    by following a trajectory offset toward the right edge of its own lane.
    """
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0

    while True:
        current_speed = self.speed if self.speed is not None else 0
        speed_error = target_speed - current_speed

        # Longitudinal control
        throttle = _lon_controller.run_step(speed_error)

        # Compute lateral offset: shift toward right edge of current lane
        # to overlap into the lane to the right (which is ego's lane)
        if self.lane is not None and hasattr(self.lane, 'rightEdge') and self.lane.rightEdge is not None:
            lane_width = self.lane.width if hasattr(self.lane, 'width') else 3.5
            offset_distance = lane_width * overlap_fraction
            # Get point on right edge at current longitudinal position
            right_edge_pt = self.lane.rightEdge.project(self.position)
            # Compute CTE relative to a virtual line offset from centerline toward right edge
            center_pt = self.lane.centerline.project(self.position)
            # Direction from center to right edge
            dx = right_edge_pt.x - center_pt.x
            dy = right_edge_pt.y - center_pt.y
            norm = (dx**2 + dy**2)**0.5
            if norm > 0:
                offset_x = center_pt.x + (dx / norm) * offset_distance
                offset_y = center_pt.y + (dy / norm) * offset_distance
                offset_pt = Vector(offset_x, offset_y)
                cte = self.position.distanceTo(offset_pt)
                # Sign: positive if we need to steer right
                cross = dx * (self.position.y - center_pt.y) - dy * (self.position.x - center_pt.x)
                if cross < 0:
                    cte = -cte
            else:
                cte = 0
        else:
            cte = self.lane.centerline.signedDistanceTo(self.position) if self.lane else 0

        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a left neighbor (adv will be in left lane, ego in right)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point in its lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial spawn point ahead in the left lane
leftLaneAheadPt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnBase = new OrientedPoint following roadDirection from leftLaneAheadPt for globalParameters.OPT_ADV_AHEAD_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnBase,
    with heading advSpawnBase.heading,
    with regionContainedIn leftLaneSec,
    with behavior AdversarialOverlapBehavior(
        target_speed=globalParameters.OPT_ADV_SPEED,
        overlap_fraction=globalParameters.OPT_LANE_OVERLAP_OFFSET
    )

require distance from egoSpawnPt to intersection >= 100
terminate when distance from ego to AdvAgent > 60