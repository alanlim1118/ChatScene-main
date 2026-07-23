"""Scenario Description:

This top-down schematic diagram illustrates a traffic scenario on a paved road marked with solid white edge lines and a dashed white center line. A blue rectangular vehicle is positioned in the left portion of the lower lane, accompanied by a straight blue arrow pointing to the right, indicating forward travel. Ahead of it in the same lane, a pink rectangular vehicle is depicted with a curved pink arrow that sweeps downward and to the left, visualizing a maneuver where the object is exiting the flow of traffic, turning, or reversing towards the left side.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(8, 12)
param ADV_SPEED = VerifaiRange(6, 10)

EGO_INIT_DIST = [30, 50]
ADV_AHEAD_DIST = [15, 25]

TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, 10):
        take SetBrakeAction(0.8)
    interrupt when withinDistanceToAnyObjs(self, 3):
        terminate

behavior AdvExitLeftBehavior():
    """
    Adversary initially follows the lane, then executes a leftward exit maneuver
    by steering toward the left edge / off-lane region.
    """
    # Phase 1: follow lane normally for a short distance
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for Range(2, 4) seconds
    
    # Phase 2: steer left to exit traffic flow
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer = 0
    target_line = self.lane.leftEdge if self.lane.leftEdge is not None else self.lane.centerline
    
    while True:
        cte = target_line.signedDistanceTo(self.position)
        speed_error = globalParameters.ADV_SPEED - (self.speed if self.speed is not None else 0)
        throttle = _lon_controller.run_step(speed_error)
        # Bias steering to the left by adding negative offset to cross-track error
        steer = _lat_controller.run_step(cte - 1.5)
        take RegulatedControlAction(throttle, steer, past_steer)
        past_steer = steer

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment with at least one lane
road = Uniform(*filter(lambda r: len(r.lanes) >= 1, network.roads))
lane = Uniform(*road.lanes)

# Ego spawn point in the left portion of the lane
egoSpawnPt = new OrientedPoint in lane.centerline

# Adversary spawn point ahead of ego in the same lane
advSpawnPt = new OrientedPoint in lane.centerline,
    ahead of egoSpawnPt by Range(ADV_AHEAD_DIST[0], ADV_AHEAD_DIST[1])

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior AdvExitLeftBehavior()

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to road.start) <= EGO_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST