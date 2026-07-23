description = "Ego vehicle on a multi-lane urban road at night is struck by a swerving truck avoiding a pedestrian from the left and pushed toward the right curb."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=10)

ego = new Car with blueprint MODEL,
	with behavior EgoBehavior()

behavior PedestrianBehavior():
    do WalkForwardBehavior(1.0)

pedestrian = new Pedestrian left of (ahead of ego by 25) by 5,
    facing 90 deg relative to ego.heading,
    with behavior PedestrianBehavior()

behavior TruckBehavior():
    do FollowLaneBehavior(target_speed=15) until (distance from self to pedestrian) < 20
    take SetSteerAction(1.0)
    take SetThrottleAction(0.5)

truck = new Truck left of (behind ego by 10) by 4,
    facing ego.heading,
    with behavior TruckBehavior()

require 20 <= (distance from ego to pedestrian) <= 30
require 7 <= (distance from ego to truck) <= 15
terminate when truck intersects ego