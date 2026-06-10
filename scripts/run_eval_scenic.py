'''
Description:
    One-time Scenic evaluation that samples scenario parameters directly from the
    ranges written in the .scenic code, WITHOUT loading the OPT-selection JSON
    produced by `train_scenario`.

    Functionally similar to scripts/run_eval.py (eval mode), except:
      - it never reads scenic_dir/<bench>/scenario_<id>.json (use_opt_json=False),
      - it generates `--num-scenes` scenes from the Scenic code ranges and runs them,
      - it writes logs/results under log/adv_train/eval_scenic/... (separate tree).

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


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp_name', type=str, default='exp')
    parser.add_argument('--output_dir', type=str, default='log')
    parser.add_argument('--ROOT_DIR', type=str, default=osp.abspath(osp.dirname(osp.dirname(osp.realpath(__file__)))))

    parser.add_argument('--max_episode_step', type=int, default=300)
    parser.add_argument('--auto_ego', action='store_true')
    # This script always runs in eval mode (runtime behavior required by ScenicRunner).
    parser.add_argument('--mode', '-m', type=str, default='eval', choices=['eval'])
    parser.add_argument('--agent_cfg', nargs='*', type=str, default=['adv_scenic.yaml'])
    parser.add_argument('--scenario_cfg', nargs='*', type=str, default=['eval_scenic.yaml'])
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
    parser.add_argument('--test_epoch', type=int, default=None)
    parser.add_argument('--num-scenes', dest='num_scenes', type=int, default=1,
                        help='Number of scenes to sample from the Scenic code ranges and evaluate.')
    args = parser.parse_args()

    # Force eval runtime mode regardless of input.
    args.mode = 'eval'

    err_list = []
    for agent_cfg in args.agent_cfg:
        for scenario_cfg in args.scenario_cfg:
            # set global parameters
            set_torch_variable(args.device)
            torch.set_num_threads(args.threads)
            set_seed(args.seed)

            # load agent config
            agent_config_path = osp.join(args.ROOT_DIR, 'safebench/agent/config', agent_cfg)
            agent_config = load_config(agent_config_path)
            agent_config['policy_name'] = args.test_policy

            ## load the corresponding model ##
            agent_config['load_dir'] = osp.join(agent_config['load_dir'], f'scenario_{args.scenario_id}')

            # load scenario config
            scenario_config_path = osp.join(args.ROOT_DIR, 'safebench/scenario/config', scenario_cfg)
            scenario_config = load_config(scenario_config_path)
            scenario_config['scenario_id'] = args.scenario_id

            # JSON-free one-time eval: sample parameters from the Scenic code ranges.
            scenario_config['use_opt_json'] = False
            scenario_config['sample_num'] = args.num_scenes
            # Ensure a single generation batch produces exactly num_scenes scenes.
            scenario_config['opt_step'] = args.num_scenes

            # Log under a dedicated `eval_scenic` tree (runtime mode stays `eval`).
            log_mode_dir = 'eval_scenic'
            args.output_dir = osp.join('log', 'adv_train', log_mode_dir, agent_config['policy_name'], f"{agent_cfg.split('.')[0]}_epoch{args.test_epoch}", f"{scenario_cfg.split('.')[0]}")
            bench_id = scenario_config.get('bench_id')
            if bench_id:
                args.output_dir = osp.join(args.output_dir, str(bench_id))
            args.exp_name = 'scenario_' + str(scenario_config['scenario_id'])
            args_dict = vars(args)
            # main entry with a selected mode
            agent_config.update(args_dict)
            print(agent_config['load_dir'])
            scenario_config.update(args_dict)
            if scenario_config['policy_type'] == 'scenic':
                from safebench.scenic_runner import ScenicRunner
                scenario_config['num_scenario'] = 1 # 'the num_scenario can only be one for scenic'
                scenario_config['route_id'] = [args.route_id]
                runner = ScenicRunner(agent_config, scenario_config)
            else:
                ## id shift due to the test settings in safebench v1
                scenario_config['route_id'] = args.route_id + 4
                runner = CarlaRunner(agent_config, scenario_config)

            # start running
            try:
                runner.run(args.test_epoch)
            except:
                runner.close()
                traceback.print_exc()
                err_list.append([agent_cfg, scenario_cfg, traceback.format_exc()])

    for err in err_list:
        print(err[0], err[1], 'failed!')
        print(err[2])
