"""Parser for self-contained scenario directories (see safebench/util/scenario_dir.py)."""

import json
import os.path as osp
from copy import deepcopy

from safebench.scenario.scenario_manager.scenario_config import ScenarioConfig
from safebench.util import scenario_dir as scenario_layout


def scenario_dir_parse(config, logger, base_extra_params):
    """
        Parse one self-contained scenario directory (see safebench/util/scenario_dir.py):
        the Scenic file, its route.pickle and its opt_params.json all live in
        config['scenario_dir']. config['scenic_file'] optionally pins a specific .scenic
        file inside that directory. Shared by the Scenic 3 (scenario_utils) and Scenic 2
        (scenario_utils_v2) parsers, which differ only in base_extra_params.
    """
    mode = config['mode']
    scenario_dir = config['scenario_dir']
    scenic_file = config.get('scenic_file') or scenario_layout.eval_scenic_file(scenario_dir)
    if scenic_file == scenario_layout.source_scenic_file(scenario_dir):
        logger.log(
            f'>> No {osp.basename(scenario_layout.route_driven_scenic_file(scenario_dir))} found; '
            f'running the original {osp.basename(scenic_file)}. Its ego is not pinned to the '
            'route spawn point, so the agent may start away from the route.', 'yellow'
        )
    logger.log(f'>> Scenario directory: {scenario_dir}')
    logger.log(f'>> Scenic file: {scenic_file}')

    route = scenario_layout.load_route(scenario_dir)
    behavior = osp.splitext(osp.basename(scenic_file))[0]

    params = {}
    use_opt_json = config.get('use_opt_json', True)
    if mode in ['eval', 'train_agent'] and use_opt_json:
        opt_path = scenario_layout.opt_params_path(scenario_dir)
        if osp.isfile(opt_path):
            with open(opt_path, 'r') as f:
                params = json.load(f)
        else:
            logger.log(f'>> {opt_path} not found; run --mode train_scenario first.', 'red')

    parsed_config = ScenarioConfig()
    parsed_config.auto_ego = config['auto_ego']
    parsed_config.num_scenario = config['num_scenario']
    parsed_config.data_id = 0
    parsed_config.scenic_file = scenic_file
    parsed_config.behavior = behavior
    parsed_config.scenario_generation_method = config['method']
    parsed_config.scenario_id = scenario_layout.scenario_id(scenario_dir)
    parsed_config.sample_num = config['sample_num']
    parsed_config.select_num = config['select_num']
    parsed_config.mode = mode
    parsed_config.opt_step = config['opt_step']
    parsed_config.scenic_mode2d = config.get('scenic_mode2d', True)
    parsed_config.use_opt_json = use_opt_json
    parsed_config.trajectory = route['trajectory']
    parsed_config.route_format = route.get('route_format', {})
    parsed_config.extra_params = dict(base_extra_params)
    parsed_config.extra_params['town'] = route['town']
    parsed_config.extra_params['weather'] = route['weather']
    parsed_config.extra_params['waypoints'] = route['waypoints']
    parsed_config.extra_params['lanePts'] = route['lanePts']
    spawnPt = route['spawnPt']
    parsed_config.extra_params['spawnPt'] = (spawnPt['x'], spawnPt['y'])
    parsed_config.extra_params['z'] = spawnPt['z']
    parsed_config.extra_params['yaw'] = spawnPt['yaw']
    if parsed_config.extra_params['weather'] is None:
        # Weather wasn't a string literal in the file; keep the file's own definition.
        del parsed_config.extra_params['weather']

    # A scenario directory holds exactly one route; route_id only names the log/JSON key.
    route_ids = config['route_id'] if config.get('route_id') is not None else [0]
    config_list = []
    for j in route_ids:
        updated_config = deepcopy(parsed_config)
        updated_config.route_id = j
        if mode in ['eval', 'train_agent'] and use_opt_json:
            key = f'OPT_{behavior}_ROUTE-{j}'
            if key not in params:
                logger.log(f'>> {key} not in opt_params.json; skipping route {j}.', 'red')
                continue
            updated_config.opt_params = params[key]
        else:
            updated_config.opt_params = None
        config_list.append(updated_config)
    return config_list
