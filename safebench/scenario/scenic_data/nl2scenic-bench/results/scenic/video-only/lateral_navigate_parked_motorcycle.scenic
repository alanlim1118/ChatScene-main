"""Scenario Description:

The ego vehicle proceeds slowly along a bustling commercial street lined with shops under clear skies, with oncoming traffic passing in the adjacent lane. A grey passenger van, initially positioned on the right side near the curb, abruptly steers into the ego vehicle's lane to avoid a parked motorcycle blocking its path. This sudden maneuver places the van directly in the ego vehicle's trajectory, causing the ego vehicle to decelerate rapidly and come to a complete stop immediately behind the van's rear bumper, culminating in a low-speed collision as the van halts in the middle of the lane.

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
VAN_MODEL = "vehicle.volkswagen.t2"
MOTO_MODEL = "vehicle.yamaha.yzf"

param OPT_EGO_SPEED = Range(2, 4)           # Ego proceeds slowly
param OPT_VAN_SPEED = Range(3, 5)           # Van initial speed before maneuver
param OPT_BRAKE_DIST = Range(6, 10)         # Distance at which ego brakes
param OPT_VAN_AHEAD_DIST = Range(25, 35)    # How far ahead the van starts relative to ego
param OPT_MOTO_OFFSET = Range(8, 12)        # Distance from van to parked motorcycle
param OPT_LANE_CHANGE_TRIGGER = Range(10, 15)  # Distance to motorcycle when van initiates lane change
OPT_STOP_DISTANCE = 0.5                     # Van stops almost completely in lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior ParkedBehavior():
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        wait

behavior VanAvoidanceBehavior(target_speed, trigger_dist, brake_amount):
    """Van follows lane then abruptly changes left to avoid motorcycle, then brakes to stop."""
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ParkedMoto <= trigger_dist)
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToLeft, target_speed=target_speed * 0.5)
        # After completing lane change, brake hard to stop in middle of ego's lane
        take SetBrakeAction(brake_amount)
        take SetThrottleAction(0)
        do WaitBehavior() for 10 seconds
    interrupt when (collision):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
        do WaitBehavior() for 10 seconds
        terminate

behavior EgoBehavior(ego_speed, brake_distance):
    """Ego follows lane slowly and brakes when obstacle is detected ahead."""
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (withinDistanceToObjsInLane(self, brake_distance)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 10 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight road section with a left adjacent lane (van moves right-to-left into ego lane)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

require len(laneSecsWithLeftLane) > 0

vanInitLaneSec = Uniform(*laneSecsWithLeftLane)
egoLaneSec = vanInitLaneSec._laneToLeft  # Ego is in the left lane; van starts in right lane

# Spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
vanProjPt = egoLaneSec.centerline.project(egoSpawnPt.position)
vanSpawnPt = new OrientedPoint following roadDirection from vanProjPt for globalParameters.OPT_VAN_AHEAD_DIST,
    with heading vanInitLaneSec.centerline.orientationAt(vanProjPt)

# Parked motorcycle ahead of van in same (right) lane, near curb
motoSpawnPt = new OrientedPoint following roadDirection from vanSpawnPt for globalParameters.OPT_MOTO_OFFSET,
    with heading vanInitLaneSec.centerline.orientationAt(vanSpawnPt)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in left lane, proceeding slowly
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

# Grey passenger van in right lane, will swerve left to avoid motorcycle
VanAgent = new Car at vanSpawnPt,
    with regionContainedIn vanInitLaneSec,
    with blueprint VAN_MODEL,
    with color "grey",
    with behavior VanAvoidanceBehavior(
        globalParameters.OPT_VAN_SPEED,
        globalParameters.OPT_LANE_CHANGE_TRIGGER,
        1.0
    )

# Parked motorcycle blocking van's path near curb
ParkedMoto = new Motorcycle at motoSpawnPt,
    with regionContainedIn vanInitLaneSec,
    with blueprint MOTO_MODEL,
    with behavior ParkedBehavior()

# Ensure scenario takes place away from intersections for clean straight-road dynamics
require distance to intersection >= 80