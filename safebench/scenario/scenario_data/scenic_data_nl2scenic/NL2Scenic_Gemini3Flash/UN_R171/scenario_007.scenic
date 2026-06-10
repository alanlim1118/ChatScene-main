"""Scenario Description:

The ego vehicle follows a lead vehicle in a straight lane for over two seconds with a lateral offset of less than one meter, 
then ego vehicle successfully completes a full lane change maneuver involving a 3.5-meter lateral displacement 
into the adjacent lane.

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

EGO_MODEL = 'vehicle.tesla.model3'
LEAD_MODEL = 'vehicle.audi.tt'

EGO_SPEED = 12
LEAD_SPEED = 10
FOLLOW_DURATION = 3 # Over two seconds
INITIAL_GAP = 15

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior():
    """Simple behavior for the lead vehicle to follow the lane."""
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

behavior EgoBehavior(target_lane_section):
    """
    Ego behavior:
    1. Follow lead vehicle in the same lane for over 2 seconds.
    2. Perform a lane change into the adjacent lane (approx 3.5m displacement).
    """
    # Phase 1: Follow the lead vehicle. 
    # Being in the same lane and using FollowLaneBehavior ensures lateral offset < 1m.
    do FollowLaneBehavior(target_speed=EGO_SPEED) for FOLLOW_DURATION seconds

    # Phase 2: Complete a full lane change maneuver.
    # In CARLA Town05, standard lane width is ~3.5m.
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_section, target_speed=EGO_SPEED)
    
    # Continue driving in the new lane for a few seconds before terminating
    do FollowLaneBehavior(target_speed=EGO_SPEED) for 5 seconds
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find all lane sections that have an adjacent lane to the left to facilitate a lane change.
# We also filter for sections that are relatively long and not part of intersections.
lane_sections_with_left = []
for lane in network.lanes:
    for section in lane.sections:
        if section.laneToLeft is not None:
            # We look for sections not in intersections to satisfy the "straight lane" description.
            if not section.lane.road.intersections:
                lane_sections_with_left.append(section)

assert len(lane_sections_with_left) > 0, "No suitable multi-lane straight road found in map."

# Sample a starting lane section
init_lane_sec = Uniform(*lane_sections_with_left)
target_lane_sec = init_lane_sec.laneToLeft

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the lead vehicle
lead_vehicle = new Car on init_lane_sec.centerline,
    with blueprint LEAD_MODEL,
    with behavior LeadVehicleBehavior()

# Spawn the ego vehicle behind the lead vehicle in the same lane
ego = new Car behind lead_vehicle by INITIAL_GAP,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(target_lane_sec)

# Ensure there is enough space ahead to perform the maneuver without hitting an intersection
require (distance to intersection) > 50
require (distance from lead_vehicle to intersection) > 50

# Safety requirement: lead vehicle should be visible to ego
require ego can see lead_vehicle