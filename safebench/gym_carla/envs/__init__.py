''' 
Date: 2023-01-31 22:23:17
LastEditTime: 2023-03-01 16:30:15
Description: 
    Copyright (c) 2022-2023 Safebench Team

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>
'''

# Lazy import: CarlaEnv pulls in scenario/YOLO stacks. External runners that only
# need route_planner or misc should not pay that cost at import time.


def __getattr__(name: str):
    if name == "CarlaEnv":
        from safebench.gym_carla.envs.carla_env import CarlaEnv

        return CarlaEnv
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
