"""Scenario Description:
Vehicle A (Adversary) is driving and brakes for stopped traffic ahead. 
Vehicle B (Ego), following behind Vehicle A, sees Vehicle A's brake lights and attempts to brake. 
Due to slick road conditions (represented by wet weather and insufficient braking distance), 
Vehicle B fails to stop in time and rear-ends Vehicle A.
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

# Weather set to Wet to represent "slick road"
param weather = 'WetNoon'

# Speed and distance constants
EGO_INIT_SPEED = Range(12, 15)
ADVA_INIT_SPEED = Range(12, 15)

# Trigger distances
ADVA_BRAKE_THRESHOLD = 15      # Distance from stopped traffic when A starts braking
EGO_BRAKE_THRESHOLD = 12       # Distance from A when B (Ego) starts braking (simulating reaction to brake lights)

# Model choices
EGO_MODEL = "vehicle.audi.etron"
ADVA_MODEL = "vehicle.tesla.model3"
STOPPED_MODEL = "vehicle.volkswagen.t2"

#################################
# AGENT BEHAVIORS               #
#################################

behavior StoppedTrafficBehavior():
    # Vehicle is already stopped
    take SetBrakeAction(1.0)
    while True:
        wait

behavior LeadVehicleABehavior(target_speed, stop_threshold):
    # Drive until close to stopped traffic, then brake hard
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(self, stop_threshold):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        # Stay braked to be hit
        while True:
            wait

behavior EgoVehicleBBehavior(target_speed, brake_threshold):
    # Drive until Lead Vehicle A brakes/gets close
    # Simulate slick road by braking but not being able to stop in time
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(self, brake_threshold):
        # B sees brake lights and brakes, but "slick road" leads to impact
        take SetThrottleAction(0)
        take SetBrakeAction(0.8) # Slightly reduced braking efficiency for "slick" effect
        while True:
            # Continue braking until collision or end of scenario
            take SetBrakeAction(0.8)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a long straight road section
lane = Uniform(*filter(lambda l: not l.intersection and l.length > 100, network.lanes))

# Starting positions along the lane
spawnPt = new OrientedPoint on lane.centerline

# Position 1: The stopped traffic
stoppedPt = new OrientedPoint following lane.centerline from spawnPt for 80

# Position 2: Vehicle A (Adversary) starts behind stopped traffic
advAPt = new OrientedPoint following lane.centerline from spawnPt for 40

# Position 3: Vehicle B (Ego) starts behind Vehicle A
egoPt = new OrientedPoint following lane.centerline from spawnPt for 10

#################################
# SCENARIO SPECIFICATION        #
#################################

# The stopped vehicle causing the chain reaction
stoppedTraffic = new Vehicle at stoppedPt,
    with blueprint STOPPED_MODEL,
    with behavior StoppedTrafficBehavior()

# Vehicle A: The lead car that brakes
advA = new Car at advAPt,
    with blueprint ADVA_MODEL,
    with behavior LeadVehicleABehavior(ADVA_INIT_SPEED, ADVA_BRAKE_THRESHOLD)

# Vehicle B: The ego car that fails to stop
ego = new Car at egoPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoVehicleBBehavior(EGO_INIT_SPEED, EGO_BRAKE_THRESHOLD)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure vehicles are in the same lane
require ego.lane == advA.lane
require advA.lane == stoppedTraffic.lane

# Terminate when a collision occurs or after some time
terminate when (distance from ego to advA) < 2.5 # Approximate distance for bumper-to-bumper collision
terminate after 20 seconds