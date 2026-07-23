"""Scenario Description:

The ego vehicle travels forward on a multi-lane urban road during the day when a white SUV merges into the lane from the right side and immediately applies its brakes, coming to a sudden halt. The ego vehicle is forced to brake urgently to avoid a crash, and the provided text indicates that a rear-end collision occurs as the ego was attempting to change lanes to the right into the path of the stopping vehicle. Following the stop, the white SUV remains stationary in the active travel lane, prompting its driver, a woman in a yellow coat, to exit the vehicle and walk around the rear to inspect the situation while traffic continues to flow in the adjacent left lane. She then returns to the driver's seat and closes the door, leaving the white vehicle stopped in the middle of the road.

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
SUV_MODEL = "vehicle.tesla.modely"
PEDESTRIAN_MODEL = "walker.pedestrian.woman"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SUV_MERGE_DIST = Range(30, 50)       # Distance ahead where SUV merges into ego lane
param OPT_SUV_BRAKE_DIST = Range(5, 10)        # Distance after merge before SUV brakes hard
param OPT_EGO_LANE_CHANGE_DIST = Range(20, 35) # Distance at which ego attempts lane change right
param OPT_WALK_INSPECT_DURATION = 6            # Seconds pedestrian inspects behind SUV
OPT_SUV_BRAKE_AMOUNT = 1.0
OPT_EGO_BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior SuvMergeAndStopBehavior(merge_dist, brake_dist, brake_amount):
    """SUV drives in right lane, merges left into ego's lane, then brakes hard."""
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED + 2) until (distance from self to ego < merge_dist)
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToLeft, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to ego < brake_dist)
        take SetThrottleAction(0), SetBrakeAction(brake_amount)
        do WaitBehavior()
    interrupt when collision:
        take SetThrottleAction(0), SetBrakeAction(brake_amount)
        do WaitBehavior()

behavior EgoDriveAndChangeRightBehavior(ego_speed, lane_change_dist, brake_amount):
    """Ego follows lane, attempts to change right, brakes if obstacle detected."""
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to SuvAgent < lane_change_dist)
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToRight, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when withinDistanceToObjsInLane(self, 8):
        take SetThrottleAction(0), SetBrakeAction(brake_amount)
        do WaitBehavior()

behavior InspectVehicleBehavior(suv_ref, inspect_duration):
    """Pedestrian exits SUV, walks around rear, inspects, returns to driver seat."""
    # Exit vehicle and walk to rear
    take SetWalkingSpeedAction(1.0)
    target_rear = new OrientedPoint behind suv_ref by 3, with heading suv_ref.heading
    do WalkToBehavior(target_rear.position)
    # Inspect: pause at rear of vehicle
    take SetWalkingSpeedAction(0)
    do WaitBehavior() for inspect_duration
    # Return to driver side
    target_driver = new OrientedPoint left of suv_ref by 1.2, with heading suv_ref.heading
    do WalkToBehavior(target_driver.position)
    # Stop at driver door (simulating re-entry)
    take SetWalkingSpeedAction(0)
    do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Find a multi-lane forward road section with both left and right adjacent lanes
candidateSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToLeft is not None and sec._laneToRight is not None:
            if sec._laneToLeft.isForward and sec._laneToRight.isForward:
                candidateSections.append(sec)

require len(candidateSections) > 0
egoLaneSec = Uniform(*candidateSections)
rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
suvSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_SUV_MERGE_DIST,
    offset laterally by 3.5  # Start in right lane

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in center lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoDriveAndChangeRightBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_LANE_CHANGE_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# White SUV starting in right lane, will merge left and brake
SuvAgent = new Car at suvSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint SUV_MODEL,
    with color "White",
    with behavior SuvMergeAndStopBehavior(
        globalParameters.OPT_SUV_MERGE_DIST,
        globalParameters.OPT_SUV_BRAKE_DIST,
        OPT_SUV_BRAKE_AMOUNT
    )

# Pedestrian (woman in yellow coat) spawns at SUV position after it stops
InspectorPedestrian = new Pedestrian at suvSpawnPt,
    with regionContainedIn None,
    with blueprint PEDESTRIAN_MODEL,
    with behavior InspectVehicleBehavior(SuvAgent, OPT_WALK_INSPECT_DURATION)

# Ensure sufficient road length ahead
require distance to intersection >= 80

terminate after 45 seconds