"""Scenario Description:

On a road marked by a central dashed white line, a blue ego vehicle travels straight forward within the lower lane. Directly ahead in the same lane, a pink adversarial vehicle executes a maneuver described as "object exiting parallel reversing to the left," visually depicted by a purple trajectory line that shows the vehicle backing up and curving upward toward the adjacent upper lane.

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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(2, 5)
param OPT_EGO_BRAKE_DISTANCE = Range(8, 15)
param OPT_ADV_REVERSE_SPEED = Range(1, 3)
param OPT_ADV_LATERAL_OFFSET = Range(3, 5)  # Lateral distance to move into upper lane
param OPT_ADV_LONGITUDINAL_OFFSET = Range(5, 10)  # How far back the adv reverses

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_EGO_BRAKE_DISTANCE):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
    terminate

behavior ReverseToLeftBehavior(reverse_speed, lateral_offset, longitudinal_offset):
    """
    Adversary reverses while steering left into the adjacent upper lane.
    This approximates 'exiting parallel reversing to the left'.
    """
    # Phase 1: Reverse with left steering to enter upper lane
    take SetGearAction(-1)  # Reverse gear
    take SetThrottleAction(reverse_speed / 5.0)
    take SetSteerAction(-0.6)  # Steer left while reversing
    
    # Monitor lateral displacement; once sufficiently in upper lane, straighten out
    initialPos = self.position
    while True:
        lateralDisplacement = abs(self.position.y - initialPos.y)
        if lateralDisplacement >= lateral_offset:
            break
        wait
    
    # Phase 2: Continue reversing straight for remaining longitudinal distance
    take SetSteerAction(0)
    reversedDistance = 0
    prevPos = self.position
    while reversedDistance < longitudinal_offset:
        currentDist = distance from self to prevPos
        reversedDistance += currentDist
        prevPos = self.position
        wait
    
    # Stop after completing the maneuver
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a left neighbor (lower lane with upper lane available)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjUpperLaneSec = egoLaneSec._laneToLeft

# Spawn ego in the lower (right) lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Spawn adversary ahead of ego in the same lane
advSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for Range(20, 35)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue ego vehicle in the lower lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),  # Blue
    with behavior EgoBehavior()

# Pink adversarial vehicle ahead in the same lane
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),  # Pink
    with behavior ReverseToLeftBehavior(
        globalParameters.OPT_ADV_REVERSE_SPEED,
        globalParameters.OPT_ADV_LATERAL_OFFSET,
        globalParameters.OPT_ADV_LONGITUDINAL_OFFSET
    )

require distance from ego to intersection >= 80 if ego.canSeeAny(network.intersections) else True
terminate when distance from ego to AdvAgent > 100 or simulation().currentTime > 30