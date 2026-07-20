'''Scenic 2.x counterpart of scenic_parse helpers in scenario_utils.py.'''

import os
import os.path as osp
import json
import pickle
from copy import deepcopy

from safebench.scenario.scenario_manager.scenario_config import ScenarioConfig


def _scenic_base_extra_params(config):
    """CARLA/Scenic 2.x globals merged into each ScenarioConfig.extra_params."""
    return {
        'port': config['port'],
        'traffic_manager_port': config['tm_port'],
        'render': 0,
        'address': '127.0.0.1',
        'timestep': config.get('fixed_delta_seconds', 0.1),
    }


def scenic_parse(config, logger):
    """
        Parse scenic config for Scenic 2.x files, especially for loading the scenic files.
    """
    mode = config['mode']
    scenic_dir = config['scenic_dir']

    route_file_formatter = osp.join(config['route_dir'], 'scenic_route.pickle')
    with open(route_file_formatter, 'rb') as f:
        data_full = pickle.load(f)

    index_path = osp.join(config['route_dir'], 'scenic_route_index.json')
    route_index = None
    if osp.exists(index_path):
        try:
            with open(index_path, 'r') as f:
                route_index = json.load(f)
        except Exception:
            route_index = None

    scenic_rel_listdir = []
    scenic_abs_listdir = []
    if config['scenario_id'] is None:
        search_scenarios = [i for i in range(1, 9)]
    else:
        search_scenarios = [config['scenario_id']]

    scenario_ids = []
    for j in search_scenarios:
        current_scenic_dir = osp.join(scenic_dir, f'scenario_{j}')
        new_files = []
        if osp.isdir(current_scenic_dir):
            new_files = sorted([path for path in os.listdir(current_scenic_dir) if path.split('.')[1] == 'scenic'])
            scenic_rel_listdir.extend(new_files)
            scenic_abs_listdir.extend(osp.join(current_scenic_dir, path) for path in new_files)
            scenario_ids.extend([j] * len(new_files))
        if (not new_files) and config.get('bench_id') is not None:
            bench_dir = osp.join(scenic_dir, str(config['bench_id']))
            if osp.isdir(bench_dir):
                bench_files = sorted([path for path in os.listdir(bench_dir) if path.split('.')[1] == 'scenic'])
                if config.get('scenario_id') is not None:
                    expected = f"scenario_{int(j):03d}.scenic"
                    if expected in bench_files:
                        bench_files = [expected]
                    else:
                        bench_files = [p for p in bench_files if p.startswith("scenario_") and p.split(".")[0].endswith(f"{int(j):03d}")]
                scenic_rel_listdir.extend(bench_files)
                scenic_abs_listdir.extend(osp.join(bench_dir, path) for path in bench_files)
                scenario_ids.extend([j] * len(bench_files))

    behaviors = [path.split('.')[0] for path in scenic_rel_listdir]
    assert len(scenic_rel_listdir) > 0, 'no scenic file in this dir'

    def _load_opt_params_for_scenario(scenario_id: int):
        bench_id = config.get('bench_id')
        candidates = []
        if bench_id:
            candidates.append(osp.join(scenic_dir, str(bench_id), f"scenario_{scenario_id}.json"))
        candidates.append(osp.join(scenic_dir, f"scenario_{scenario_id}", f"scenario_{scenario_id}.json"))
        for p in candidates:
            try:
                with open(p, 'r') as f:
                    return json.load(f)
            except Exception:
                continue
        return {}

    try:
        params = _load_opt_params_for_scenario(config['scenario_id'])
    except Exception:
        params = {}

    config_list = []
    for i, scenic_file in enumerate(scenic_abs_listdir):
        parsed_config = ScenarioConfig()
        parsed_config.auto_ego = config['auto_ego']
        parsed_config.num_scenario = config['num_scenario']
        parsed_config.data_id = i
        parsed_config.scenic_file = scenic_file
        parsed_config.behavior = behaviors[i]
        parsed_config.scenario_generation_method = config['method']
        parsed_config.scenario_id = scenario_ids[i]

        try:
            params = _load_opt_params_for_scenario(scenario_ids[i])
        except Exception:
            params = {}

        parsed_config.sample_num = config['sample_num']
        parsed_config.trajectory = []
        parsed_config.select_num = config['select_num']
        parsed_config.mode = config['mode']
        parsed_config.opt_step = config['opt_step']
        parsed_config.use_opt_json = config.get('use_opt_json', True)
        parsed_config.extra_params = _scenic_base_extra_params(config)

        route = config['route_id']
        if route is None:
            parsed_config.route_id = None
            if mode in ['eval', 'train_agent'] and config.get('use_opt_json', True):
                parsed_config.opt_params = params[f'OPT_{behaviors[i]}']
            else:
                parsed_config.opt_params = None
            config_list.append(parsed_config)
        else:
            for j in route:
                updated_config = deepcopy(parsed_config)
                updated_config.route_id = j
                data_key = f'scenario_id_{updated_config.scenario_id}_route_id_{j}'
                if data_key not in data_full and route_index is not None:
                    bench_id = config.get('bench_id')
                    if bench_id is None:
                        bench_id = osp.basename(osp.dirname(updated_config.scenic_file))
                    idx_key = f'{bench_id}:{updated_config.scenario_id}:{j}'
                    idx_entry = route_index.get(idx_key)
                    if isinstance(idx_entry, dict):
                        maybe_key = idx_entry.get('pickle_key')
                        if isinstance(maybe_key, str):
                            data_key = maybe_key
                data = data_full[data_key]
                updated_config.trajectory = data['trajectory']
                updated_config.route_format = data.get('route_format', {})
                updated_config.extra_params['town'] = data['town']
                updated_config.extra_params['weather'] = data['weather']
                updated_config.extra_params['waypoints'] = data['waypoints']
                updated_config.extra_params['lanePts'] = data['lanePts']
                spawnPt = data['spawnPt']
                updated_config.extra_params['spawnPt'] = (spawnPt['x'], spawnPt['y'])
                updated_config.extra_params['z'] = spawnPt['z']
                updated_config.extra_params['yaw'] = spawnPt['yaw']

                if mode in ['eval', 'train_agent'] and config.get('use_opt_json', True):
                    try:
                        updated_config.opt_params = params[f'OPT_{behaviors[i]}_ROUTE-{j}']
                    except Exception:
                        continue
                else:
                    updated_config.opt_params = None
                config_list.append(updated_config)

    config_by_map = {}
    for cfg in config_list:
        if cfg.extra_params['town'] not in config_by_map:
            config_by_map[cfg.extra_params['town']] = []
        config_by_map[cfg.extra_params['town']].append(cfg)
    new_lists = list(config_by_map.values())
    config_list = [item for sublist in new_lists for item in sublist]
    return config_list
