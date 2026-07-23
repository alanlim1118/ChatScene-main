"""Scenario Description:

The ego car travels straight forward within the middle lane. Ahead of it, the adversarial object moves across the lane, illustrating the scenario of a lead object entering from the right.

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
param OPT_ADV_START_DIST = Range(20, 35)  # Distance ahead of ego where adv starts crossing
param OPT_BRAKE_DIST = Range(8, 12)      # Distance at which ego brakes for adv

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior CrossLaneFromRightBehavior(target_speed=3):
    """
    Adversarial agent crosses from the right lane into the ego's lane,
    then continues following the ego's lane centerline.
    """
    # First, move laterally from right lane into ego lane
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0
    
    # Phase 1: Cross into ego lane by following ego lane centerline from right lane start position
    target_line = egoLaneSec.centerline
    crossing_complete = False
    
    while not crossing_complete:
        current_speed = self.speed if self.speed is not None else 0
        cte = target_line.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed
        
        throttle = _lon_controller.run_step(speed_error)
        current_steer_angle = _lat_controller.run_step(cte)
        
        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle
        
        # Check if we've reached the ego lane centerline (within tolerance)
        if abs(cte) < 0.5:
            crossing_complete = True
    
    # Phase 2: Continue following the ego lane
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both left and right neighbors (middle lanes)
laneSecsWithBothNeighbors = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            laneSecsWithBothNeighbors.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithBothNeighbors)
rightLaneSec = egoLaneSec._laneToRight

# Ego spawn point in the middle lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial spawn point: ahead of ego in the right lane
rightLaneAheadPt = rightLaneSec.centerline.project(
    egoSpawnPt.position.offsetRotated(egoSpawnPt.heading, Vector(0, globalParameters.OPT_ADV_START_DIST))
)
advSpawnPt = new OrientedPoint at rightLaneAheadPt,
    with heading egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior CrossLaneFromRightBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

require distance from egoSpawnPt to intersection >= 80
terminate when distance from ego to AdvAgent > 60