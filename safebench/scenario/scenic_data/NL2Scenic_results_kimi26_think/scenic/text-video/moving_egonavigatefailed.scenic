"""Scenario Description:

The ego vehicle travels forward on a multi-lane urban road during the day when a white SUV merges into the lane from the right side and immediately applies its brakes, coming to a sudden halt. The ego vehicle is forced to brake urgently to avoid a crash, and the provided text indicates that a rear-end collision occurs as the ego vehicle was attempting to change lanes to the right into the path of the stopping vehicle. Following the stop, the white SUV remains stationary in the active travel lane, prompting its driver, a woman in a yellow coat, to exit the vehicle and walk around the rear to inspect the situation while traffic continues to flow in the adjacent left lane. She then returns to the driver's seat and closes the door, leaving the white vehicle stopped in the middle of the road.

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
SUV_MODEL = "vehicle.audi.etron"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SUV_SPEED = Range(8, 10)
param OPT_SUV_DIST = Range(40, 55)
param OPT_LANE_CHANGE_TRIGGER = Range(30, 40)
param OPT_BRAKE_DIST = Range(4, 7)
param OPT_TRAFFIC_SPEED = Range(8, 12)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior MergeAndStop():
    # Merge briefly then emergency stop
    do FollowLaneBehavior(target_speed=globalParameters.OPT_SUV_SPEED) for 0.8 seconds
    while True:
        take SetThrottleAction(0), SetBrakeAction(1)

behavior EgoBehavior(lane_change_target, adv_agent, lane_change_trigger, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to adv_agent < lane_change_trigger)
        do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to adv_agent < brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds

behavior InspectVehicle(suv):
    # Wait until the SUV has come to a stop
    while suv.speed > 0.5:
        wait

    # Approach the stopped SUV
    take SetWalkingDirectionAction(angle from self to suv), SetWalkingSpeedAction(1.2)
    while distance from self to suv > 3.0:
        wait

    # Walk around the rear of the vehicle
    take SetWalkingDirectionAction(suv.heading + 180 deg), SetWalkingSpeedAction(1.2)
    do WaitBehavior() for 3 seconds

    # Return toward the driver's door
    take SetWalkingDirectionAction(suv.heading), SetWalkingSpeedAction(1.2)
    do WaitBehavior() for 3 seconds

    take SetWalkingSpeedAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

# Find forward lanes that have a valid forward lane to the right
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
adjLaneSec = egoLaneSec._laneToRight

# Spawn point for the SUV in the right lane, ahead of ego
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
SUVSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_SUV_DIST

# Spawn point for background traffic in the adjacent left lane
trafficSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for -25

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- White SUV in the right lane (merges and stops) ---
suv = new Car at SUVSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with blueprint SUV_MODEL,
    with color (1, 1, 1),
    with behavior MergeAndStop()

# --- Ego vehicle in the left lane, changes right toward the SUV ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        adjLaneSec,
        suv,
        globalParameters.OPT_LANE_CHANGE_TRIGGER,
        globalParameters.OPT_BRAKE_DIST
    )

# --- Pedestrian (driver) inspects rear of the stopped SUV ---
driverDoorPt = new OrientedPoint left of suv by 1.2
driver = new Pedestrian at driverDoorPt,
    with heading suv.heading,
    with regionContainedIn None,
    with behavior InspectVehicle(suv)

# --- Background traffic flowing in the adjacent left lane ---
trafficCar = new Car at trafficSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRAFFIC_SPEED)

require distance to intersection >= 80
terminate after 45 seconds