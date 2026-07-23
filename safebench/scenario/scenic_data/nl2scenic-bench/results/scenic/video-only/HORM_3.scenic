"""Scenario Description:

An aerial view depicts a suburban traffic scenario where a dark-colored ego vehicle travels along an on-ramp merging from the right onto a multi-lane main road. Ahead on the ramp, another dark vehicle leads the way, while on the main road to the left, a white car followed by a red truck travels in the adjacent lane. As the ego vehicle proceeds, it merges onto the highway, successfully positioning itself ahead of the white car and red truck, which remain in the lane to the rear-left. The road is flanked by green grassy areas, dense clusters of trees on the right, and residential houses with brown roofs on the left.

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
LEAD_RAMP_MODEL = "vehicle.tesla.model3"
WHITE_CAR_MODEL = "vehicle.audi.a2"
RED_TRUCK_MODEL = "vehicle.carlamotors.firetruck"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_RAMP_SPEED = Range(8, 12)
param OPT_MAIN_LANE_SPEED = Range(8, 12)
param OPT_RAMP_LEAD_DISTANCE = Range(15, 25)
param OPT_MAIN_CAR_OFFSET = Range(-10, 5)
param OPT_TRUCK_FOLLOW_DISTANCE = Range(12, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior RampMergeBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=8):
        take SetBrakeAction(1)

behavior MainLaneCruiseBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=10):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable merge section: a lane that has a successor (merge point)
# and a left neighbor representing the main road
mergeCandidateLanes = []
for lane in network.lanes:
    if lane.successor is not None and lane._laneToLeft is not None:
        if lane._laneToLeft.isForward:
            mergeCandidateLanes.append(lane)

require len(mergeCandidateLanes) > 0
rampLane = Uniform(*mergeCandidateLanes)
mainLane = rampLane._laneToLeft

# Define spawn points
rampStartPt = new OrientedPoint on rampLane.centerline
leadRampPt = follow roadDirection from rampStartPt for resample(OPT_RAMP_LEAD_DISTANCE)

# Position main lane vehicles relative to ego's longitudinal position but in the left lane
mainLaneRefPt = new OrientedPoint on mainLane.centerline,
    with heading mainLane.centerline.headingAt(rampStartPt.position)
whiteCarPt = follow roadDirection from mainLaneRefPt for resample(OPT_MAIN_CAR_OFFSET)
redTruckPt = follow roadDirection from whiteCarPt for -resample(OPT_TRUCK_FOLLOW_DISTANCE)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle on the ramp
ego = new Car at rampStartPt,
    with blueprint EGO_MODEL,
    with color "black",
    with behavior RampMergeBehavior(globalParameters.OPT_EGO_SPEED),
    with regionContainedIn None

# Leading dark vehicle ahead on the ramp
leadRampCar = new Car at leadRampPt,
    with blueprint LEAD_RAMP_MODEL,
    with color "dark_blue",
    with behavior RampMergeBehavior(globalParameters.OPT_LEAD_RAMP_SPEED),
    with regionContainedIn None

# White car on the main road to the left
whiteCar = new Car at whiteCarPt,
    with blueprint WHITE_CAR_MODEL,
    with color "white",
    with behavior MainLaneCruiseBehavior(globalParameters.OPT_MAIN_LANE_SPEED),
    with regionContainedIn None

# Red truck following the white car on the main road
redTruck = new Car at redTruckPt,
    with blueprint RED_TRUCK_MODEL,
    with color "red",
    with behavior MainLaneCruiseBehavior(globalParameters.OPT_MAIN_LANE_SPEED),
    with regionContainedIn None

# Ensure proper spatial configuration
require distance from rampStartPt to leadRampPt >= 10
require distance from whiteCarPt to redTruckPt >= 8

# Terminate after ego has merged and traveled some distance on the main road
terminate when (distance from ego to rampLane.centerline.end > 30) or simulation().currentTime > 30