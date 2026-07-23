"""Scenario Description:

A vehicle is depicted leaving a parked position in a rural area at night under clear weather conditions, angling out from a parking space adjacent to a designated handicapped spot marked with a wheelchair symbol. An arrow indicates the car's forward trajectory as it moves into the lane, where it encounters a dog standing directly in its path at a non-junction area. The scene is presented as a top-down schematic diagram showing the vehicle, the animal, and the road markings including vertical lines separating parking bays.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural map with parking areas
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
DOG_MODEL = "walker.dog"

param EGO_SPEED = Range(2, 4)
param BRAKE_DIST = Range(8, 12)
param PARKING_ANGLE_OFFSET = Range(30, 50)  # Angle to pull out of parking
param DOG_DISTANCE_AHEAD = Range(15, 25)    # Distance ahead on lane where dog stands

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior PullOutAndDriveBehavior(target_speed, brake_distance):
    # Initially pull out at an angle then follow lane
    try:
        do FollowLaneBehavior(target_speed)
    interrupt when withinDistanceToObjsInLane(self, brake_distance):
        take SetThrottleAction(0), SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        terminate

behavior StandStillBehavior():
    while True:
        take SetWalkingSpeedAction(0)
        wait

#################################
# ENVIRONMENT                   #
#################################

param timeOfDay = 22  # Night time (10 PM)
param weather = Weather(precipitation=0, cloudiness=0, windIntensity=0)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-junction road segment with adjacent parking
roadSegment = Uniform(*filter(lambda s: not s.isIntersection and len(s.lanes) > 0, network.roadSegments))
targetLane = Uniform(*roadSegment.lanes)

# Define parking area adjacent to the lane
parkingRegion = targetLane.rightEdge.offsetBy(3)

# Handicapped spot reference point (adjacent to ego parking spot)
handicapSpotPt = new OrientedPoint in parkingRegion,
    with heading targetLane.heading + 90 deg

# Ego starts in parking space next to handicap spot, angled outward
egoParkingPt = new OrientedPoint left of handicapSpotPt by 3,
    with heading targetLane.heading + globalParameters.PARKING_ANGLE_OFFSET

# Dog placement point on the lane centerline ahead of where ego will enter
dogPlacementBase = new OrientedPoint in targetLane.centerline
dogSpawnPt = new OrientedPoint following targetLane.orientation from dogPlacementBase for globalParameters.DOG_DISTANCE_AHEAD

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoParkingPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior PullOutAndDriveBehavior(globalParameters.EGO_SPEED, globalParameters.BRAKE_DIST)

# Handicapped spot marker (using a static object to represent the wheelchair symbol area)
handicapMarker = new Object at handicapSpotPt,
    with regionContainedIn None,
    with width 2.5,
    with length 5,
    with color (0, 0, 1)  # Blue marking for handicapped spot

# Dog standing directly in the ego's path on the lane
dog = new Pedestrian at dogSpawnPt,
    with blueprint DOG_MODEL,
    with heading targetLane.heading,
    with regionContainedIn None,
    with behavior StandStillBehavior()

# Ensure dog is placed at a non-junction area and within reasonable distance
require not dog.isOnIntersection
require distance from ego to dog >= 10

terminate after 30 seconds