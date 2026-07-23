"""Scenario Description:

Under dark nighttime conditions, the ego vehicle approaches and executes a left turn at an urban T-junction. As the ego vehicle enters the intersection from the southern approach, it navigates around cross-traffic on the perpendicular road. An adversary vehicle traveling from the left arm proceeds straight across the intersection towards the right. Concurrently, traffic from the right arm involves multiple agents; two vehicles pass straight through from right to left, while a third vehicle from the right arm executes a left turn. The scene is characterized by low visibility, with vehicle positions and movements primarily defined by their headlights and taillights against the dark asphalt.

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
NPC_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(4, 7)
param ADV_SPEED = Range(5, 8)
param NPC_SPEED = Range(4, 7)

param EGO_INIT_DIST = Range(30, 45)
param ADV_INIT_DIST = Range(25, 40)
param NPC_INIT_DIST = Range(20, 35)

param BRAKE_DIST = Range(8, 15)

CONST_HEADING_TOL = 25 deg

#################################
# MONITORS                      #
#################################

monitor NightAndLights():
    setWeather(darkNight())
    while True:
        for agent in objects:
            if agent is Car:
                setVehicleLightState(agent, "Position|LowBeam")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior AdvStraightBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    terminate

behavior NpcStraightBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.NPC_SPEED, trajectory=trajectory)
    terminate

behavior NpcLeftTurnBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.NPC_SPEED, trajectory=trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-junction where ego comes from south (approx heading ~0 deg / northbound)
tJunctions = filter(lambda i: i.isTJunction, network.intersections)
intersection = Uniform(*tJunctions)

# Ego: left turn from southern incoming lane
egoCandidateManeuvers = filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*egoCandidateManeuvers)
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: straight from left arm to right arm (relative to ego's perspective)
# Left arm relative to ego heading means ~90 deg offset
advCandidateManeuvers = filter(
    lambda m: m.type is ManeuverType.STRAIGHT and 
              abs(relativeHeading(m.startLane.centerline[0], egoSpawnPt.heading) - 90 deg) < CONST_HEADING_TOL,
    intersection.maneuvers
)
advManeuver = Uniform(*advCandidateManeuvers)
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Right arm NPCs: straight from right to left (~270 deg or -90 deg relative to ego)
rightStraightManeuvers = filter(
    lambda m: m.type is ManeuverType.STRAIGHT and
              abs(relativeHeading(m.startLane.centerline[0], egoSpawnPt.heading) + 90 deg) < CONST_HEADING_TOL,
    intersection.maneuvers
)
npcStraightManeuver1 = Uniform(*rightStraightManeuvers)
npcStraightManeuver2 = Uniform(*rightStraightManeuvers)

npcStraight1InitLane = npcStraightManeuver1.startLane
npcStraight1Trajectory = [npcStraight1InitLane, npcStraightManeuver1.connectingLane, npcStraightManeuver1.endLane]
npcStraight1SpawnPt = new OrientedPoint in npcStraight1InitLane.centerline

npcStraight2InitLane = npcStraightManeuver2.startLane
npcStraight2Trajectory = [npcStraight2InitLane, npcStraightManeuver2.connectingLane, npcStraightManeuver2.endLane]
npcStraight2SpawnPt = new OrientedPoint in npcStraight2InitLane.centerline

# Right arm NPC: left turn from right arm
rightLeftManeuvers = filter(
    lambda m: m.type is ManeuverType.LEFT_TURN and
              abs(relativeHeading(m.startLane.centerline[0], egoSpawnPt.heading) + 90 deg) < CONST_HEADING_TOL,
    intersection.maneuvers
)
npcLeftManeuver = Uniform(*rightLeftManeuvers)
npcLeftInitLane = npcLeftManeuver.startLane
npcLeftTrajectory = [npcLeftInitLane, npcLeftManeuver.connectingLane, npcLeftManeuver.endLane]
npcLeftSpawnPt = new OrientedPoint in npcLeftInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

require monitor NightAndLights()

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with regionContainedIn None

adversary = new Car at advSpawnPt,
    with blueprint NPC_MODEL,
    with behavior AdvStraightBehavior(advTrajectory),
    with regionContainedIn None

npcStraight1 = new Car at npcStraight1SpawnPt,
    with blueprint NPC_MODEL,
    with behavior NpcStraightBehavior(npcStraight1Trajectory),
    with regionContainedIn None

npcStraight2 = new Car at npcStraight2SpawnPt,
    with blueprint NPC_MODEL,
    with behavior NpcStraightBehavior(npcStraight2Trajectory),
    with regionContainedIn None

npcLeftTurn = new Car at npcLeftSpawnPt,
    with blueprint NPC_MODEL,
    with behavior NpcLeftTurnBehavior(npcLeftTrajectory),
    with regionContainedIn None

# Distance constraints
require globalParameters.EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= globalParameters.EGO_INIT_DIST[1]
require globalParameters.ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= globalParameters.ADV_INIT_DIST[1]
require globalParameters.NPC_INIT_DIST[0] <= (distance from npcStraight1SpawnPt to intersection) <= globalParameters.NPC_INIT_DIST[1]
require globalParameters.NPC_INIT_DIST[0] <= (distance from npcStraight2SpawnPt to intersection) <= globalParameters.NPC_INIT_DIST[1]
require globalParameters.NPC_INIT_DIST[0] <= (distance from npcLeftSpawnPt to intersection) <= globalParameters.NPC_INIT_DIST[1]

# Ensure NPCs on right arm are staggered to avoid overlap
require (distance from npcStraight1SpawnPt to npcStraight2SpawnPt) >= 10
require (distance from npcStraight1SpawnPt to npcLeftSpawnPt) >= 10
require (distance from npcStraight2SpawnPt to npcLeftSpawnPt) >= 10

terminate when (distance from ego to egoSpawnPt) > 80