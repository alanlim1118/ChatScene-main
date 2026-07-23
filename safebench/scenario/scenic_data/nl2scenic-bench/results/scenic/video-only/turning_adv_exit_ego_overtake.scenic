"""Scenario Description:

The ego vehicle travels forward on a straight, two-lane residential road lined with uniform houses featuring red roofs under an overcast sky, following a grey sedan at a moderate speed. As the ego vehicle closes the distance, the lead grey sedan slows down and initiates a left turn to exit the road, crossing the lane. Simultaneously, the ego vehicle attempts to overtake the slowing sedan, but the turning vehicle cuts directly into its path, resulting in a sudden side-impact collision. After the impact, the ego vehicle slows significantly to a crawl, continuing forward on the now-clear road while passing pedestrians walking on the sidewalk to the right.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.domains.driving.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_CAR_MODEL = "vehicle.tesla.model3"
LEAD_CAR_COLOR = "128,128,128"  # Grey

INITIAL_FOLLOW_DIST = Range(25, 35)
LEAD_NORMAL_SPEED = Range(8, 12)       # Moderate speed m/s
LEAD_SLOW_SPEED = Range(2, 4)          # Slowing for turn
EGO_OVERTAKE_SPEED = Range(10, 14)     # Ego tries to pass
EGO_CRAWL_SPEED = Range(1, 2)          # Post-collision crawl
TURN_TRIGGER_DIST = Range(18, 25)      # Distance at which lead car begins left turn
OVERTAKE_TRIGGER_DIST = Range(12, 18)  # Distance at which ego attempts overtake
COLLISION_CHECK_DIST = 3               # Proximity threshold for side-impact detection
POST_COLLISION_TIME = 15               # Seconds to continue crawling after impact
PEDESTRIAN_WALK_SPEED = Range(0.8, 1.4)
NUM_PEDESTRIANS = Range(2, 4)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadCarBehavior():
    """Lead grey sedan drives forward, then slows and turns left across ego's path."""
    try:
        do FollowLaneBehavior(speed=LEAD_NORMAL_SPEED)
    interrupt when (distance from self to ego <= TURN_TRIGGER_DIST):
        take SetThrottleAction(0), SetBrakeAction(0.6)
        wait
        # Initiate left turn across the lane
        do TurnLeftBehavior(speed=LEAD_SLOW_SPEED)

behavior EgoOvertakeAndCrawlBehavior():
    """Ego follows lane, attempts overtake when lead slows, then crawls after collision."""
    collision_occurred = False
    try:
        do FollowLaneBehavior(speed=LEAD_NORMAL_SPEED)
    interrupt when (distance from self to leadCar <= OVERTAKE_TRIGGER_DIST):
        # Attempt to overtake by moving left and accelerating
        do ChangeLaneBehavior(direction='left', speed=EGO_OVERTAKE_SPEED)
    
    # Monitor for side-impact collision with lead car during overtake
    try:
        do FollowLaneBehavior(speed=EGO_OVERTAKE_SPEED)
    interrupt when (distance from self to leadCar <= COLLISION_CHECK_DIST):
        # Side-impact detected: brake hard briefly then crawl
        take SetThrottleAction(0), SetBrakeAction(1)
        wait
        collision_occurred = True
    
    if collision_occurred:
        do FollowLaneBehavior(speed=EGO_CRAWL_SPEED) for POST_COLLISION_TIME seconds
    else:
        do FollowLaneBehavior(speed=EGO_OVERTAKE_SPEED)

behavior SidewalkWalkBehavior():
    """Pedestrians walk along the right sidewalk."""
    do WalkAlongSidewalkBehavior(speed=PEDESTRIAN_WALK_SPEED)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Select a straight two-lane road segment
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 2 and s.isStraight, network.roadSegments))
egoLane = Uniform(*filter(lambda l: l is not s.leftmostLane, roadSegment.lanes))

# Place ego vehicle in the right lane of the two-lane road
egoSpawnPt = new OrientedPoint in egoLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EgoOvertakeAndCrawlBehavior()

# Place lead grey sedan ahead of ego in the same lane
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by INITIAL_FOLLOW_DIST,
    on egoLane.centerline

leadCar = new Car at leadSpawnPt,
    with blueprint LEAD_CAR_MODEL,
    with color LEAD_CAR_COLOR,
    with regionContainedIn None,
    with behavior LeadCarBehavior()

require distance from ego to leadCar >= INITIAL_FOLLOW_DIST - 5

# Place pedestrians on the right sidewalk
rightSidewalk = egoLane.rightSidewalk
for i in range(NUM_PEDESTRIANS):
    pedOffset = Range(10, 60) + (i * Range(8, 15))
    pedSpawnPt = new OrientedPoint ahead of egoSpawnPt by pedOffset,
        on rightSidewalk.centerline
    new Pedestrian at pedSpawnPt,
        with regionContainedIn None,
        with behavior SidewalkWalkBehavior()

# Environmental conditions: overcast sky
param weather = Weather(preset='Overcast')

terminate after 45 seconds