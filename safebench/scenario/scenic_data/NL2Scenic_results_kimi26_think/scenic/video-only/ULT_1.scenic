"""Scenario Description:

The ego vehicle travels northbound on a straight road toward a four-way intersection, following a leading adversary that intends to turn left. As the leading vehicle enters the intersection and positions itself for the left turn, it must yield to an opposing adversary traveling straight southbound from the opposite arm. Consequently, the ego vehicle decelerates and comes to a complete stop behind the leading vehicle, waiting for the opposing traffic to pass and the intersection to clear.

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

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [25, 35]
LEADING_ADV_INIT_DIST = [5, 15]
OPPOSING_ADV_INIT_DIST = [20, 30]

param EGO_SPEED = VerifaiRange(5, 10)
param LEADING_ADV_SPEED = VerifaiRange(3, 6)
param OPPOSING_ADV_SPEED = VerifaiRange(5, 10)

param SAFETY_DIST = VerifaiRange(8, 15)
param YIELD_DIST = VerifaiRange(12, 20)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

TERM_DIST = 70

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(leadingAdversary, 100):
            setClosestTrafficLightStatus(leadingAdversary, "green")
        if withinDistanceToTrafficLight(opposingAdversary, 100):
            setClosestTrafficLightStatus(opposingAdversary, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, 2):
        terminate

behavior LeadingAdvBehavior(