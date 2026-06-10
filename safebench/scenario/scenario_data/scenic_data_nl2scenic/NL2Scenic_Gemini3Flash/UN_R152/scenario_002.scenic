"""Scenario Description:

The subject vehicle (ego) follows a forward vehicle (lead) along a lane. 
As they reach an intersection, the forward vehicle performs a turn (left or right),
while the subject vehicle continues straight through the intersection.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_SPEED = Range(7, 9)
param LEAD_DISTANCE = Range(10, 15)

# Weather options
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'WetNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Car models
carModels = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.dodge.charger_police', 'vehicle.lincoln.mkz_2017', 'vehicle.mercedes.coupe', 'vehicle.mini.cooper_s', 'vehicle.toyota.prius', 'vehicle.volkswagen.t2']

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoFollowingBehavior(trajectory, target_speed=10):
    """Behavior for the ego vehicle to follow a trajectory while avoiding collisions."""
    try:
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
    interrupt when withinDistanceToAnyCars(self, 8):
        # Brake if the lead vehicle slows down for the turn
        take SetBrakeAction(0.8)
        take SetThrottleAction(0)
    
behavior LeadTurnBehavior(trajectory, target_speed=8):
    """Behavior for the lead vehicle to perform its turn."""
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Filter lanes that lead into an intersection and have both a straight and a turning maneuver.
# This ensures both vehicles start in the same lane but can diverge.
valid_lanes = []
for l in network.lanes:
    mans = l.maneuvers
    has_straight = any(m.type == ManeuverType.STRAIGHT for m in mans)
    has_turn = any(m.type in [ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN] for m in mans)
    if has_straight and has_turn:
        valid_lanes.append(l)

# 2. Randomly select a valid starting lane and the specific maneuvers.
target_lane = Uniform(*valid_lanes)
ego_maneuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, target_lane.maneuvers))
lead_maneuver = Uniform(*filter(lambda m: m.type in [ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN], target_lane.maneuvers))

# 3. Helper to define the sequence of lanes for FollowTrajectoryBehavior.
def get_trajectory_lanes(maneuver):
    if maneuver.connectingLane:
        return [maneuver.startLane, maneuver.connectingLane, maneuver.endLane]
    else:
        return [maneuver.startLane, maneuver.endLane]

ego_traj = get_trajectory_lanes(ego_maneuver)
lead_traj = get_trajectory_lanes(lead_maneuver)

# 4. Define spawn points relative to the intersection entry.
# intersection_entry is the end of the incoming lane's centerline.
intersection_entry_pt = target_lane.centerline[-1]

# The lead vehicle starts closer to the intersection.
lead_spawn_pt = OrientedPoint following roadDirection from intersection_entry_pt for -15

# The ego starts behind the lead vehicle.
ego_spawn_pt = OrientedPoint following roadDirection from lead_spawn_pt for -1 * globalParameters.LEAD_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the lead vehicle.
leadVehicle = new Car at lead_spawn_pt,
    with blueprint Uniform(*carModels),
    with behavior LeadTurnBehavior(trajectory=lead_traj, target_speed=globalParameters.OPT_LEAD_SPEED)

# Spawn the ego vehicle.
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint 'vehicle.tesla.model3',
    with behavior EgoFollowingBehavior(trajectory=ego_traj, target_speed=globalParameters.OPT_EGO_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure vehicles start in the correct spatial relationship.
require (distance from ego to leadVehicle) < 20
require ego.lane == leadVehicle.lane

# Terminate when the ego has crossed the intersection.
terminate when (distance from ego to intersection_entry_pt) > 30 and ego.speed > 0.1