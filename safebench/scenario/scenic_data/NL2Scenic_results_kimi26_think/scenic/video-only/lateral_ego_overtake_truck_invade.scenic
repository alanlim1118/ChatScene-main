"""Scenario Description:

The ego vehicle travels on a multi-lane roadway on a clear morning, approaching an underpass beneath a concrete overpass. A large cargo truck carrying a load of white panels is driving in the adjacent right lane. As the ego vehicle proceeds forward, attempting to overtake the truck near the entrance of the underpass, the large truck unexpectedly drifts to the left. This movement causes the truck to invade the ego vehicle's lane, resulting in a sudden side-impact collision between the two vehicles as they pass under the bridge.

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
TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param OPT_EGO_SPEED = Range(10, 14)
param OPT_TRUCK_SPEED = Range(5, 8)
param OPT_INITIAL_DIST = Range(25, 45)
param OPT_DRIFT_TRIGGER_DIST = Range(8, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior TruckBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED) until (distance from self to ego < globalParameters.OPT_DRIFT_TRIGGER_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_TRUCK_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToRight is not None and
            laneSec._laneToRight.isForward
        ):
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
truckLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
truckLanePt = truckLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following roadDirection from truckLanePt for globalParameters.OPT_INITIAL_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

Truck = new Car at truckSpawnPt,
    with heading truckSpawnPt.heading,
    with regionContainedIn truckLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior TruckBehavior()

require distance to intersection >= 100
terminate when (distance from ego to egoSpawnPt) > 150