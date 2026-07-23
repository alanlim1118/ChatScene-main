"""Scenario Description:

In this traffic scenario, a blue ego car travels straight along the lower lane of a two-lane road, moving towards the right as indicated by a blue directional arrow. It approaches a pink adversarial object, depicted as a circle located below the road, which is laterally moving upwards and crossing into the ego-traffic area from the right side of the vehicle's path, indicated by a pink arrow pointing perpendicular to the flow of traffic.

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
param OPT_ADV_SPEED = Range(1.5, 3.5)
param OPT_BRAKE_DIST = Range(8, 15)
param OPT_CROSS_TRIGGER_DIST = Range(20, 35)
param OPT_LATERAL_OFFSET = Range(4, 7)  # Distance below/right of road for adversarial spawn

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior LateralCrossingBehavior(reference_actor, trigger_distance, cross_speed):
    # Wait until ego is close enough before starting to cross
    while distance from self to reference_actor > trigger_distance:
        wait
    # Move laterally (perpendicular to ego's heading) across the road
    take SetWalkingDirectionAction(reference_actor.heading + 90 deg)
    take SetWalkingSpeedAction(cross_speed)
    # Continue crossing until well past the ego lane or timeout
    do FollowLaneBehavior(target_speed=cross_speed) for 10 seconds
    take SetWalkingSpeedAction(0)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find straight lanes suitable for the scenario
straightLanes = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None:
            straightLanes.append(laneSec)

egoLaneSec = Uniform(*straightLanes)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversarial spawn point: to the right/below the ego's path, offset laterally
# "below the road" and "from the right side" means positive lateral offset relative to ego heading
advSpawnBase = new OrientedPoint at egoSpawnPt,
    with heading egoSpawnPt.heading
advSpawnPt = new OrientedPoint right of advSpawnBase by globalParameters.OPT_LATERAL_OFFSET,
    with heading egoSpawnPt.heading + 90 deg  # Facing upward/perpendicular to traffic flow

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color "0,0,255",  # Blue ego car
    with behavior EgoBehavior()

AdvAgent = new Pedestrian at advSpawnPt,
    with heading egoSpawnPt.heading + 90 deg,  # Perpendicular to ego, moving upward
    with regionContainedIn None,
    with color "255,105,180",  # Pink adversarial agent
    with behavior LateralCrossingBehavior(ego, globalParameters.OPT_CROSS_TRIGGER_DIST, globalParameters.OPT_ADV_SPEED)

require distance from ego to AdvAgent >= 30  # Ensure sufficient initial separation
terminate when distance from ego to AdvAgent > 80 or simulation().currentTime > 30