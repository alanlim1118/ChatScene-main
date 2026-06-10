"""Scenario Description:

The lead vehicle performs an emergency lane change to avoid a stopped car when its own Time-to-Collision (TTC) 
with that car is only 1.5 seconds. This creates a high-urgency "late reveal" situation for the ego vehicle 
following closely behind the lead vehicle.

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

EGO_SPEED = 15
LEAD_SPEED = 15
TTC_THRESHOLD = 1.5
GAP_EGO_TO_LEAD = Range(10, 15)

# Weather setup
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(target_lane, obstacle):
    """
    Follows the lane until the TTC to the obstacle is below the threshold,
    then performs an emergency lane change.
    """
    try:
        do FollowLaneBehavior(target_speed=LEAD_SPEED)
    interrupt when (self.speed > 0) and (distance from self to obstacle) <= (self.speed * TTC_THRESHOLD):
        # Perform emergency lane change
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=LEAD_SPEED)
        # Continue driving in the new lane
        do FollowLaneBehavior(target_speed=LEAD_SPEED)

behavior EgoVehicleBehavior():
    """
    Follows the lane at a constant speed, simulating a late reaction 
    to the revealed obstacle.
    """
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior StoppedCarBehavior():
    """
    Stays stationary.
    """
    take SetBrakeAction(1.0)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane section that has an adjacent lane to the left for the emergency maneuver
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # Ensure it is a forward lane and has a left lane that is also forward
        if laneSec.isForward and laneSec.laneToLeft is not None and laneSec.laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, "No suitable lane sections found for lane change scenario."

# Select the starting lane section
initLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = initLaneSec.laneToLeft

# Define spawn points
# We place the stopped car first, then work backwards
stoppedCarSpawnPt = new OrientedPoint on initLaneSec.centerline

# Lead car starts further back to gain speed
leadCarSpawnPt = new OrientedPoint following roadDirection from stoppedCarSpawnPt for -60

# Ego car starts behind the lead car
egoCarSpawnPt = new OrientedPoint following roadDirection from leadCarSpawnPt for -GAP_EGO_TO_LEAD

#################################
# SCENARIO SPECIFICATION        #
#################################

# The obstacle vehicle that is stopped in the lane
stopped_car = new Car at stoppedCarSpawnPt,
    with behavior StoppedCarBehavior()

# The lead vehicle that will swerve late
lead_vehicle = new Car at leadCarSpawnPt,
    with behavior LeadVehicleBehavior(leftLaneSec, stopped_car)

# The ego vehicle following the lead vehicle
ego = new Car at egoCarSpawnPt,
    with rolename 'hero',
    with behavior EgoVehicleBehavior()

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure the scenario starts well away from intersections to avoid traffic light interference
require (distance from stopped_car to intersection) > 20
require (distance from lead_vehicle to intersection) > 20

# Terminate when the ego vehicle passes the stopped car or a collision occurs
terminate when (relative heading of (angle to stopped_car) from ego.heading) > 90 deg 
               or (distance from ego to stopped_car) > 100