"""Scenario Description:

Under overcast daylight conditions, the ego vehicle proceeds along a two-lane road bordered by green construction barriers and leafless trees, approaching a marked crosswalk and intersection. A black Volkswagen Passat travels directly ahead, its brake lights activating as it decelerates sharply near the pedestrian crossing. Reacting to the sudden stop, the ego vehicle engages in emergency braking and steers slightly to the right to evade a direct rear-end collision, but the lead sedan simultaneously drifts toward the right lane boundary, causing a minor impact between the two vehicles. The incident occurs adjacent to a large blue institutional archway on the right, where a dark SUV is positioned near the intersection, while construction cranes are visible in the background beyond the left-side fencing.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
LEAD_MODEL = 'vehicle.volkswagen.passat'
SUV_MODEL = 'vehicle.tesla.modelx'

param EGO_SPEED = VerifaiRange(8, 12)
param LEAD_SPEED = VerifaiRange(8, 12)
param LEAD_BRAKE_DIST = VerifaiRange(15, 25)
param EGO_REACT_DIST = VerifaiRange(8, 14)
param DRIFT_OFFSET = VerifaiRange(0.3, 0.6)

TERM_DIST = 80
CRASH_DIST = 2.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(brake_dist, drift_offset):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)
    interrupt when (distance from self to crosswalkRegion) <= brake_dist:
        take SetBrakeAction(1.0)
        take SetSteeringAction(drift_offset)
        do HoldBehavior() for 10 seconds
        abort

behavior EgoEmergencyBehavior(react_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (distance from self to leadCar) <= react_dist:
        take SetBrakeAction(1.0)
        take SetSteeringAction(-0.15)
        do HoldBehavior() for 10 seconds
        abort

behavior HoldBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(12, 18),
    with heading egoSpawnPt.heading

crosswalkRegion = egoInitLane.crosswalks[0] if egoInitLane.crosswalks else egoInitLane.centerline.pointAtDistance(30)

suvSpawnPt = new OrientedPoint right of intersection.center by Range(8, 12),
    with heading egoSpawnPt.heading + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

# Environmental parameters for overcast daylight
param weather = Weather(preset='Overcast', sun_altitude=45)

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoEmergencyBehavior(globalParameters.EGO_REACT_DIST)

leadCar = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with color '0,0,0',
    with behavior LeadBehavior(globalParameters.LEAD_BRAKE_DIST, globalParameters.DRIFT_OFFSET)

suv = new Car at suvSpawnPt,
    with blueprint SUV_MODEL,
    with color '30,30,30',
    with behavior HoldBehavior()

require 25 <= (distance from egoSpawnPt to intersection) <= 45
require distance from leadSpawnPt to crosswalkRegion >= 10
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
terminate when (distance from ego to leadCar) <= CRASH_DIST