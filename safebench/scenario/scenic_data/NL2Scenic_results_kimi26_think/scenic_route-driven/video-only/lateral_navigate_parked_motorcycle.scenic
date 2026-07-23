"""Scenario Description:

The ego vehicle proceeds slowly along a bustling commercial street lined with shops under clear skies, with oncoming traffic passing in the adjacent lane. A grey passenger van, initially positioned on the right side near the curb, abruptly steers into the ego vehicle's lane to avoid a parked motorcycle blocking its path. This sudden maneuver places the van directly in the ego vehicle's trajectory, causing the ego vehicle to decelerate rapidly and come to a complete stop immediately behind the van's rear bumper, culminating in a low-speed collision as the van halts in the middle of the lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
VAN_MODEL = "vehicle.mercedes.sprinter"
MOTO_MODEL = "vehicle.harley-davidson.low_rider"

param OPT_VAN_SPEED = Range(2, 4)
param OPT_VAN_AHEAD_DIST = Range(8, 12)
param OPT_MOTO_AHEAD_DIST = Range(4, 6)
param OPT_MOTO_TRIGGER_DIST = Range(2, 4)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior VanBehavior(moto, target_lane, van_speed, trigger_dist):
    # Follow the right lane until the blocking motorcycle is close
    do FollowLaneBehavior(target_speed=van_speed) until (distance from self to moto <= trigger_dist)
    # Abruptly steer into the ego lane to avoid the motorcycle
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=van_speed)
    # Halt in the middle of the lane
    while True:
        take SetBrakeAction(1)
        take SetThrottleAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight

# Van in the right lane (near curb), ahead of ego by the same distance the
# original placed ego behind it
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
vanSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_VAN_AHEAD_DIST

# Motorcycle parked ahead of the van in the same right lane
motoSpawnPt = new OrientedPoint following roadDirection from vanSpawnPt for globalParameters.OPT_MOTO_AHEAD_DIST

intersection = Uniform(*network.intersections)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (left lane, proceeding slowly) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

# --- Parked motorcycle blocking the van's path (right lane) ---
Motorcycle = new Motorcycle at motoSpawnPt,
    with heading motoSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint MOTO_MODEL,
    with behavior WaitBehavior()

# --- Grey passenger van (right lane near curb) ---
VanAgent = new Car at vanSpawnPt,
    with heading vanSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint VAN_MODEL,
    with color Color(0.5, 0.5, 0.5),
    with behavior VanBehavior(Motorcycle, egoLaneSec, globalParameters.OPT_VAN_SPEED, globalParameters.OPT_MOTO_TRIGGER_DIST)

require distance to intersection >= 50
