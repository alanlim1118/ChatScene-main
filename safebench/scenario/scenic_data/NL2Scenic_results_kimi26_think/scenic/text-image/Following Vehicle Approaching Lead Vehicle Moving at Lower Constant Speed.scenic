"""Scenario Description:

In a top-down schematic view of a traffic scenario, a vehicle travels straight along a lane in an urban area during daylight with clear weather conditions. The road segment is a non-junction with a posted speed limit of 55 mph or more, delineated by a dotted lane marking above and a solid road edge below. The subject vehicle, depicted as a larger, boxy vehicle in the rear, is following a lead vehicle, which appears as a smaller car ahead in the same lane. Both vehicles display arrows indicating forward movement, and the situation involves the rear vehicle closing in on the lead vehicle, which is maintaining a lower constant speed.

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

EGO_MODEL = "vehicle.carlamotors.carlacola"  # larger, boxy vehicle
LEAD_MODEL = "vehicle.audi.a2"               # smaller car

param OPT_EGO_SPEED = Range(24, 28)        # ~55-60 mph in m/s
param OPT_LEAD_SPEED = Range(12, 16)       # lower constant speed in m/s
param OPT_LEAD_DISTANCE = Range(25, 40)    # initial gap to lead vehicle (m)
param OPT_BRAKE_DISTANCE = Range(10, 15)   # threshold distance to begin braking (m)

#################################
# AGENT BEHAVIORS               #
#################################

behavior CloseInBehavior(speed, brake_distance):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_distance):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Gather forward lane sections for straight, non-junction travel
laneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            laneSecs.append(laneSec)

egoLaneSec = Uniform(*laneSecs)

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following egoSpawnPt.heading from egoSpawnPt for globalParameters.OPT_LEAD_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior CloseInBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

LeadAgent = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint LEAD_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)

require distance to intersection >= 50
terminate when distance from ego to egoSpawnPt > 150