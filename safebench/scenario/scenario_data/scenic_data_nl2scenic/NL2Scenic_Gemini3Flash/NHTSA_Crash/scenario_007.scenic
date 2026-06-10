"""Scenario Description:

A northbound vehicle, A, was stopped waiting at a red traffic signal in an urban area on a major artery. 
Another vehicle, B, coming from some distance behind, didn't notice that A was stopped and could not stop in time.

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

# Selection of weather for an urban setting
WEATHER_OPTIONS = ('ClearNoon', 'CloudyNoon', 'ClearSunset')
param weather = Uniform(*WEATHER_OPTIONS)

# Speed for vehicle B (approx 50 km/h)
B_SPEED = Range(13, 15)
# Distance between B and A initially
START_DISTANCE = Range(40, 60)

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    """Keeps the traffic light red for the duration of the scenario."""
    freezeTrafficLights()
    while True:
        # Get the traffic light affecting vehicle A
        setClosestTrafficLightStatus(vehicleA, "red")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior StopAtSignal():
    """Behavior for Vehicle A to remain stationary at the red light."""
    while True:
        take SetBrakeAction(1.0), SetThrottleAction(0)

behavior InattentiveDrive(target_speed):
    """Behavior for Vehicle B: follows the lane without collision avoidance to simulate 'not noticing'."""
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a signalized intersection in an urban area (major artery often has multiple lanes)
signalized_intersections = filter(lambda i: i.isSignalized and len(i.incomingLanes) >= 2, network.intersections)
inter = Uniform(*signalized_intersections)

# Filter for a Northbound lane (Heading approx 90 degrees)
# In Scenic-CARLA, 0 is East, 90 is North, 180 is West, 270 is South
northbound_lanes = filter(lambda l: 70 < l.heading < 110, inter.incomingLanes)
target_lane = Uniform(*northbound_lanes)

# Place Vehicle A at the stop line of the intersection
spawn_pt_A = target_lane.centerline.back

#################################
# SCENARIO SPECIFICATION        #
#################################

# Vehicle A: The stopped vehicle
vehicleA = new Car at spawn_pt_A,
    facing target_lane.heading,
    with behavior StopAtSignal()

# Vehicle B: The approaching vehicle (Ego)
ego = new Car behind vehicleA by START_DISTANCE,
    with rolename 'hero',
    facing target_lane.heading,
    with behavior InattentiveDrive(B_SPEED)

# Ensure the traffic light stays red
require monitor TrafficLights()

# Terminate scenario upon collision or after a reasonable time
terminate when distance from ego to vehicleA < 2
terminate after 15 seconds