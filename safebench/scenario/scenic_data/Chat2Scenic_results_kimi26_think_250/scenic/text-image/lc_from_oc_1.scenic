description = "Ego vehicle changes lanes from left to right with a leading vehicle in the target lane and oncoming traffic in the departure lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) for 3 seconds
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior LeadingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

leadingCar = new Car ahead of (right of ego by Range(3.5, 4.0)) by Range(15, 25),
    with regionContainedIn egoLaneSec._laneToRight,
    with behavior LeadingBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, is_oppositeTraffic=True)

adv = new Car ahead of ego by Range(40, 60),
    with regionContainedIn egoLaneSec,
    with behavior AdversaryBehavior()

require (distance from ego to leadingCar) >= 15
require (distance from ego to adv) >= 40
terminate when ego in egoLaneSec._laneToRight