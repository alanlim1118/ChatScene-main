"""Scenario Description:

In this top-down schematic labeled "Encroaching, Oncoming Vehicle Test Scenario," a subject vehicle outlined in green travels straight along a highway lane bordered by a dashed white line on the right and a solid yellow line on the left. An oncoming vehicle, depicted with a white outline in the opposing lane to the left of the yellow line, is shown angling sharply across the solid yellow divider, effectively drifting into the subject vehicle's lane of travel. This visual representation corresponds to a test case where an ADS-equipped vehicle must detect and react to an opposing vehicle that has lost its lane position and is encroaching into the subject vehicle's path, creating a potential head-on collision scenario.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(10, 15)         # Highway speed (m/s)
param OPT_ADV_SPEED = Range(10, 15)         # Oncoming speed (m/s)
param OPT_ENCROACH_DIST = Range(80, 120)    # Distance along road to adversary start
param OPT_BRAKE_DIST = Range(25, 40)        # Distance at which ego begins to brake
param OPT_CROSS_DEPTH = Range(60, 100)      # How far past the encroach point the adv drives

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior FollowLineBehavior(line, target_speed=10):
    """
    Follows the given PolylineRegion (line) using longitudinal and latitudinal controllers.
    """
    assert line is not None
    assert isinstance(line, PolylineRegion)

    distanceToEndpoint = 5  # meters
    end_point = line[-1]  # Last point of the PolylineRegion

    # Instantiate controllers
    _lon_controller, _lat_controller = simulation