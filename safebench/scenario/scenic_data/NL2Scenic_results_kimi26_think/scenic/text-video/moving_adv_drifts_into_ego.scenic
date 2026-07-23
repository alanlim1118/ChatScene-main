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
LEAD_MODEL = 'vehicle.audi.a2'      # sedan surrogate for the VW Passat
SUV_MODEL = 'vehicle.jeep.wrangler'

param EGO_SPEED = Range(8, 12)
param LEAD_SPEED = Range(6, 9)
param LEAD_BRAKE_DIST = Range(10, 15)
param SAFETY_DIST = Range(10, 15)
CRASH_DIST = 2.5
TERM_DIST = 60

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until (distance from self to intersection) <= globalParameters.LEAD_BRAKE_DIST
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        take SetSteerAction(0.25)
        wait

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(1.0)
        take SetSteerAction(0.2)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead vehicle spawn point ahead on the same approach lane
leadSpawnPt = new OrientedPoint in egoInitLane.centerline

# Dark SUV positioned on the right near the intersection
intersectionApproach = new OrientedPoint at egoInitLane.centerline.end
suvSpawnPt = new OrientedPoint right of intersectionApproach by 4,
    with heading egoInitLane.centerline.end.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Lead sedan decelerating sharply near the crosswalk
lead = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with behavior LeadBehavior()

# Dark SUV near the intersection on the right
suv = new Car at suvSpawnPt,
    with blueprint SUV_MODEL,
    with heading egoInitLane.centerline.end.heading,
    with regionContainedIn None,
    with behavior WaitBehavior()

# Spatial requirements
require 20 <= (distance from egoSpawnPt to leadSpawnPt) <= 30
require (distance from leadSpawnPt to intersection) < (distance from egoSpawnPt to intersection)
require 30 <= (distance to intersection) <= 50
terminate when (distance to egoSpawnPt) > TERM_DIST