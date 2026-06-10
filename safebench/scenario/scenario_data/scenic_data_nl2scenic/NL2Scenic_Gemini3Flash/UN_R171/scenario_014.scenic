"""Scenario Description:

The ego vehicle travels along a straight road at a constant speed and approaches a slower moving 
heavy truck or motorcycle from behind that is traveling at a speed exactly 50 km/h less than the ego vehicle.

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

# Speed definitions in km/h converted to m/s
EGO_SPEED_KPH = 80
DIFF_KPH = 50
OTHER_SPEED_KPH = EGO_SPEED_KPH - DIFF_KPH

EGO_SPEED_MS = EGO_SPEED_KPH / 3.6
OTHER_SPEED_MS = OTHER_SPEED_KPH / 3.6

# Starting distance between vehicles
INITIAL_DISTANCE = Range(30, 50)

# Blueprints from the provided list
carModels = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt', 'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.dodge.charger_police', 'vehicle.jeep.wrangler_rubicon', 'vehicle.lincoln.mkz_2017', 'vehicle.mercedes.coupe', 'vehicle.mini.cooper_s', 'vehicle.ford.mustang', 'vehicle.nissan.micra', 'vehicle.nissan.patrol', 'vehicle.seat.leon', 'vehicle.tesla.model3', 'vehicle.toyota.prius', 'vehicle.volkswagen.t2']
motorcycleModels = ['vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja', 'vehicle.yamaha.yzf']
truckModels = ['vehicle.carlamotors.carlacola', 'vehicle.tesla.cybertruck']

# Select leading vehicle blueprint and determine its class
lead_model = Uniform(*truckModels, *motorcycleModels)

#################################
# AGENT BEHAVIORS               #
#################################

behavior DriveAtConstantSpeed(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a long straight road section (not near an intersection)
straight_lanes = filter(lambda l: not l.intersection, network.lanes)
selected_lane = Uniform(*straight_lanes)

# Define spawn point for the ego vehicle
ego_spawn_pt = new OrientedPoint on selected_lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn ego vehicle
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint Uniform(*carModels),
    with behavior DriveAtConstantSpeed(EGO_SPEED_MS)

# Spawn leading vehicle (Truck or Motorcycle) ahead of ego
if lead_model in truckModels:
    lead_vehicle = new Truck following roadDirection from ego for INITIAL_DISTANCE,
        with blueprint lead_model,
        with behavior DriveAtConstantSpeed(OTHER_SPEED_MS)
else:
    lead_vehicle = new Motorcycle following roadDirection from ego for INITIAL_DISTANCE,
        with blueprint lead_model,
        with behavior DriveAtConstantSpeed(OTHER_SPEED_MS)

# Ensure both are on a straight stretch with enough room
require (distance from ego to intersection) > 60
require (distance from lead_vehicle to intersection) > 60

# Termination condition (optional)
terminate when (distance from ego to lead_vehicle) < 5 or (distance from ego to ego_spawn_pt) > 150