"""Scenario Description:

The ego vehicle is driving forward under wet weather conditions on an elevated highway or bridge, approaching a motorcycle carrying two riders ahead in the same lane. As the ego vehicle gets closer, the motorcyclist loses control on the slick road surface and falls over, dropping the bike and both riders directly into the ego vehicle's path. The ego vehicle is forced into an emergency braking scenario but ultimately results in a rear-end collision with the fallen two-wheeled vehicle and its occupants lying on the road.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
MOTO_MODEL = "vehicle.yamaha.yzf"

param OPT_EGO_SPEED = Range(8, 12)        # Ego speed in m/s (~30-43 km/h)
param OPT_MOTO_SPEED = Range(6, 9)        # Motorcycle speed before fall
param OPT_INITIAL_GAP = Range(30, 50)     # Initial distance between ego and motorcycle
param OPT_FALL_DISTANCE = Range(15, 25)   # Distance from ego when motorcycle falls
param OPT_BRAKE_DELAY = Range(0.3, 0.8)   # Reaction delay for ego braking (seconds)

#################################
# AGENT BEHAVIORS               #
#################################

behavior MotorcycleFallBehavior(fall_distance):
    """Motorcycle rides normally then suddenly falls over when ego is within fall_distance."""
    do FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED) until (distance from self to ego < fall_distance)
    # Simulate loss of control: brake hard and set steering to simulate falling
    take SetBrakeAction(1)
    take SetThrottleAction(0)
    take SetSteerAction(Uniform(-1, 1))  # Random steer to simulate uncontrolled fall
    wait  # Remain stationary on road after falling

behavior EgoEmergencyBrakeBehavior(brake_delay):
    """Ego follows lane until motorcycle falls, then brakes after reaction delay."""
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvMoto < globalParameters.OPT_FALL_DISTANCE + 5):
        # Simulate human reaction delay before braking
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) for brake_delay seconds
        take SetBrakeAction(1)
        take SetThrottleAction(0)
        wait  # Continue braking until simulation ends

#################################
# SPATIAL RELATIONS             #
#################################

# Select a suitable straight road section on an elevated highway/bridge area
# Town04 has elevated highway sections; filter for forward lanes with sufficient length
candidateLanes = []
for lane in network.lanes:
    if lane.isForward and lane.length > 80:
        for section in lane.sections:
            if section.length > 60:
                candidateLanes.append(section)

require len(candidateLanes) > 0
egoLaneSec = Uniform(*candidateLanes)

# Place ego vehicle on the selected lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place motorcycle ahead of ego in the same lane
motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_INITIAL_GAP

require network.laneSectionAt(motoSpawnPt) is egoLaneSec

#################################
# WEATHER CONDITIONS            #
#################################

# Wet weather conditions: heavy rain, wet road, low visibility
weather = WeatherConditions(
    precipitation=Range(0.6, 1.0),
    precipitationDeposits=Range(0.5, 0.9),
    windIntensity=Range(0.3, 0.6),
    fogDensity=Range(0.1, 0.3),
    wetness=Range(0.7, 1.0)
)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoEmergencyBrakeBehavior(globalParameters.OPT_BRAKE_DELAY)

AdvMoto = new Motorcycle at motoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint MOTO_MODEL,
    with behavior MotorcycleFallBehavior(globalParameters.OPT_FALL_DISTANCE)

# Ensure the scenario takes place on an elevated section (z-coordinate check for Town04 highway)
require egoSpawnPt.position.z > 5

# Terminate scenario after collision or timeout
terminate after 15 seconds