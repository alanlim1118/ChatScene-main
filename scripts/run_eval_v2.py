'''
Description:
    Scenic 2.x-only entry point for SafeBench scenic evaluation.

    Mirrors scripts/run_eval.py but uses ScenicRunnerV2 and scenic_utils_v2.
    Requires a Python env with Scenic 2.x (e.g. scenic2-venv with bundled Scenic/)
    plus SafeBench deps (torch, carla, pygame).

    Supports train_scenario (OPT selection) and eval only.

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>
'''
import setGPU
import traceback
import os.path as osp

import torch

from safebench.util.run_util import load_config
from safebench.util.torch_util import set_seed, set_torch_variable
from safebench.carla_runner import CarlaRunner
from safebench.util import scenario_dir as scenario_layout


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp_name', type=str, default='exp')
    parser.add_argument('--output_dir', type=str, default='log')
    parser.add_argument('--ROOT_DIR', type=str, default=osp.abspath(osp.dirname(osp.dirname(osp.realpath(__file__)))))

    parser.add_argument('--max_episode_step', type=int, default=300)
    parser.add_argument('--auto_ego', action='store_true')
    parser.add_argument('--mode', '-m', type=str, default='eval', choices=['train_scenario', 'eval'])
    parser.add_argument('--agent_cfg', nargs='*', type=str, default=['adv_scenic.yaml'])
    parser.add_argument('--scenario_cfg', nargs='*', type=str, default=None,
                        help='default: eval_scenic_scenario_dir.yaml with --scenario_dir, else eval_scenic_v2.yaml')
    parser.add_argument('--continue_agent_training', '-cat', type=bool, default=False)
    parser.add_argument('--continue_scenario_training', '-cst', type=bool, default=False)

    parser.add_argument('--seed', '-s', type=int, default=0)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--device', type=str, default='cuda:0' if torch.cuda.is_available() else 'cpu')

    parser.add_argument('--num_scenario', '-ns', type=int, default=2, help='num of scenarios we run in one episode')
    parser.add_argument('--save_video', action='store_true')
    parser.add_argument('--render', type=bool, default=True)
    parser.add_argument('--frame_skip', '-fs', type=int, default=1, help='skip of frame in each step')
    parser.add_argument('--port', type=int, default=2002, help='port to communicate with carla')
    parser.add_argument('--tm_port', type=int, default=8002, help='traffic manager port')
    parser.add_argument('--fixed_delta_seconds', type=float, default=0.1)
    parser.add_argument('--test_policy', type=str, default='sac')
    parser.add_argument('--route_id', type=int, default=0)
    parser.add_argument('--scenario_id', type=int, default=0)
    parser.add_argument('--scenario_dir', type=str, default=None,
                        help='Self-contained scenario: a <id>/ directory holding <id>.scenic + route.pickle '
                             '(or a .scenic file inside one). Replaces --scenario_id and the YAML route_dir/scenic_dir/bench_id.')
    parser.add_argument('--test_epoch', type=int, default=None)
    parser.add_argument('--bench_id', type=str, default=None,
                        help='Override bench_id from scenario config (subdirectory under scenic_dir)')
    args = parser.parse_args()
    if args.scenario_cfg is None:
        args.scenario_cfg = ['eval_scenic_scenario_dir.yaml' if args.scenario_dir else 'eval_scenic_v2.yaml']

    scenario_dir, scenic_file, scenario_name = None, None, args.scenario_id
    if args.scenario_dir is not None:
        scenario_dir, scenic_file = scenario_layout.resolve(args.scenario_dir)
        scenario_name = scenario_layout.scenario_id(scenario_dir)

    err_list = []
    for agent_cfg in args.agent_cfg:
        for scenario_cfg in args.scenario_cfg:
            set_torch_variable(args.device)
            torch.set_num_threads(args.threads)
            set_seed(args.seed)

            agent_config_path = osp.join(args.ROOT_DIR, 'safebench/agent/config', agent_cfg)
            agent_config = load_config(agent_config_path)
            agent_config['policy_name'] = args.test_policy

            agent_config['load_dir'] = osp.join(agent_config['load_dir'], f'scenario_{scenario_name}')

            scenario_config_path = osp.join(args.ROOT_DIR, 'safebench/scenario/config', scenario_cfg)
            scenario_config = load_config(scenario_config_path)
            scenario_config['scenario_id'] = args.scenario_id
            if args.bench_id is not None:
                scenario_config['bench_id'] = args.bench_id

            args.output_dir = osp.join('log', 'adv_train', args.mode, agent_config['policy_name'], f"{agent_cfg.split('.')[0]}_epoch{args.test_epoch}", f"{scenario_cfg.split('.')[0]}")
            if scenario_dir is not None:
                # Self-contained scenario dir: log under <dataset>/<scenario_id>/, ignore YAML bench layout.
                args.output_dir = osp.join(args.output_dir, scenario_layout.dataset_name(scenario_dir))
                args.exp_name = scenario_name
            else:
                bench_id = scenario_config.get('bench_id')
                if bench_id:
                    args.output_dir = osp.join(args.output_dir, str(bench_id))
                args.exp_name = 'scenario_' + str(scenario_config['scenario_id'])
            args_dict = vars(args)
            agent_config.update(args_dict)
            print(agent_config['load_dir'])
            scenario_config.update(args_dict)
            if scenario_dir is not None:
                # After update(): args_dict carries the raw --scenario_dir/--scenario_id values.
                scenario_config.update(scenario_id=scenario_name, scenario_dir=scenario_dir, scenic_file=scenic_file)
            if scenario_config['policy_type'] == 'scenic':
                from safebench.scenic_runner_v2 import ScenicRunnerV2
                scenario_config['num_scenario'] = 1
                scenario_config['route_id'] = [args.route_id]
                runner = ScenicRunnerV2(agent_config, scenario_config)
            else:
                scenario_config['route_id'] = args.route_id + 4
                runner = CarlaRunner(agent_config, scenario_config)

            try:
                runner.run(args.test_epoch)
            except Exception:
                runner.close()
                traceback.print_exc()
                err_list.append([agent_cfg, scenario_cfg, traceback.format_exc()])

    for err in err_list:
        print(err[0], err[1], 'failed!')
        print(err[2])

    import sys
    sys.exit(1 if err_list else 0)
