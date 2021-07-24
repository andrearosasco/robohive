import numpy as np
from gym import utils, spaces
from mjrl.envs import mujoco_env
from mujoco_py import MjViewer
from mj_envs.utils.quatmath import *
import os
from mj_envs.utils.obj_vec_dict import ObsVecDict
import collections

ADD_BONUS_REWARDS = True
OBS_KEYS = ['hand_jnt', 'obj_vel', 'palm_pos', 'obj_pos', 'obj_rot', 'target_pos'] # DAPG
RWD_KEYS = ['palm_obj', 'smooth', 'bonus', 'tool_target'] # DAPG

#OBS_KEYS = ['hand_jnt', 'obj_vel', 'palm_pos', 'obj_pos', 'obj_rot', 'target_pos', 'nail_impact', 'tool_pos', 'goal_pos', 'hand_vel']

RWD_MODE = 'dense' # dense/ sparse

class ToolsEnvV0(mujoco_env.MujocoEnv, utils.EzPickle, ObsVecDict):
    def __init__(self, *args, **kwargs):
        # get sim
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        self.sims = []
        self.sims_idx = []
        for i, path in enumerate(kwargs['model_paths']) : 
            self.sims.append(mujoco_env.get_sim(model_path=path))
            self.sims_idx.append(i)
        sim = self.sims[0]
        # ids
        self.target_obj_sid = sim.model.site_name2id('S_target')
        self.S_grasp_sid = sim.model.site_name2id('S_grasp')
        self.obj_bid = sim.model.body_name2id('object')
        self.tool_sid = sim.model.site_name2id('tool')

        # change actuator sensitivity
        for sim in self.sims : 
            sim.model.actuator_gainprm[sim.model.actuator_name2id('A_WRJ1'):sim.model.actuator_name2id('A_WRJ0')+1,:3] = np.array([10, 0, 0])
            sim.model.actuator_gainprm[sim.model.actuator_name2id('A_FFJ3'):sim.model.actuator_name2id('A_THJ0')+1,:3] = np.array([1, 0, 0])
            sim.model.actuator_biasprm[sim.model.actuator_name2id('A_WRJ1'):sim.model.actuator_name2id('A_WRJ0')+1,:3] = np.array([0, -10, 0])
            sim.model.actuator_biasprm[sim.model.actuator_name2id('A_FFJ3'):sim.model.actuator_name2id('A_THJ0')+1,:3] = np.array([0, -1, 0])
            # scales
            self.act_mid = np.mean(sim.model.actuator_ctrlrange, axis=1)
            self.act_rng = 0.5*(sim.model.actuator_ctrlrange[:,1]-sim.model.actuator_ctrlrange[:,0])
        sim = self.sims[0]

        # get env
        utils.EzPickle.__init__(self)
        ObsVecDict.__init__(self)
        self.obs_dict = {}
        self.rwd_dict = {}
        mujoco_env.MujocoEnv.__init__(self, sim=sim, frame_skip=5)
        self.action_space = spaces.Box(-1.0*np.ones_like(self.action_space.low), np.ones_like(self.action_space.high), dtype=np.float32)

    # step the simulation forward
    def step(self, a):
        # apply action and step
        a = np.clip(a, -1.0, 1.0)
        a = self.act_mid + a*self.act_rng
        self.do_simulation(a, self.frame_skip)

        # observation and rewards
        obs = self.get_obs()
        self.expand_dims(self.obs_dict) # required for vectorized rewards calculations
        self.rwd_dict = self.get_reward_dict(self.obs_dict)
        self.squeeze_dims(self.rwd_dict)
        self.squeeze_dims(self.obs_dict)

        # finalize step
        env_info = self.get_env_infos()
        return obs, env_info['rwd_'+RWD_MODE], bool(env_info['done']), env_info

    def get_reward_dict(self, obs_dict):
        # get to hammer
        palm_obj_dist = np.linalg.norm(obs_dict['palm_pos'] - obs_dict['obj_pos'], axis=-1)
        # take hammer head to nail
        tool_target_dist = np.linalg.norm(obs_dict['tool_pos'] - obs_dict['target_pos'], axis=-1)
        # make nail go inside
        # target_goal_dist = np.linalg.norm(obs_dict['target_pos'] - obs_dict['goal_pos'], axis=-1)
        # vel magnitude (handled differently in DAPG)
        hand_vel_mag = np.linalg.norm(obs_dict['hand_vel'], axis=-1)
        obj_vel_mag = np.linalg.norm(obs_dict['obj_vel'], axis=-1)
        # lifting tool
        lifted = (obs_dict['obj_pos'][:,:,2] > 0.04) * (obs_dict['tool_pos'][:,:,2] > 0.04)
        solved = tool_target_dist < 0.05

        rwd_dict = collections.OrderedDict((
            # Optional Keys
            # ('finger_dist', -0.1*finger_dist),
            ('palm_obj', - 0.1 * palm_obj_dist),
            ('tool_target', -5.0 * tool_target_dist),
            # ('target_goal', -10.0 * target_goal_dist),
            ('smooth', -1e-2 * (hand_vel_mag + obj_vel_mag)),
            # ('bonus', 2.0*lifted + 25.0*(target_goal_dist<0.020) + 75.0*(target_goal_dist<0.010)),
            ('bonus', 5*lifted),
            # Must keys
            # ('sparse',  -1.0*target_goal_dist),
            ('sparse',  1.0*lifted),
            # ('solved',  target_goal_dist<0.010),
            ('solved',  solved),
            ('done',    palm_obj_dist > 1.0),
        ))
        rwd_dict['dense'] = np.sum([rwd_dict[key] for key in RWD_KEYS], axis=0)
        return rwd_dict

    def get_obs(self):
        # qpos for hand, xpos for obj, xpos for target
        self.obs_dict['t'] = np.array([self.sim.data.time])
        self.obs_dict['hand_jnt'] = self.data.qpos[:-6].copy()
        self.obs_dict['obj_vel'] = np.clip(self.data.qvel[-6:].copy(), -1.0, 1.0)
        self.obs_dict['palm_pos'] = self.data.site_xpos[self.S_grasp_sid].copy()
        self.obs_dict['obj_pos'] = self.data.body_xpos[self.obj_bid].copy()
        self.obs_dict['obj_rot'] = quat2euler(self.data.body_xquat[self.obj_bid].copy())
        self.obs_dict['target_pos'] = self.data.site_xpos[self.target_obj_sid].copy()

        # keys missing from DAPG-env but needed for rewards calculations
        self.obs_dict['tool_pos'] = self.data.site_xpos[self.tool_sid].copy()
        # self.obs_dict['goal_pos'] = self.data.site_xpos[self.goal_sid].copy()
        self.obs_dict['hand_vel'] = np.clip(self.data.qvel[:-6].copy(), -1.0, 1.0)

        t, obs = self.obsdict2obsvec(self.obs_dict, OBS_KEYS)
        return obs

    # use latest obs, rwds to get all info (be careful, information belongs to different timestamps)
    # Its getting called twice. Once in step and sampler calls it as well
    def get_env_infos(self):
        env_info = {
            'time': self.obs_dict['t'][()],
            'rwd_dense': self.rwd_dict['dense'][()],
            'rwd_sparse': self.rwd_dict['sparse'][()],
            'solved': self.rwd_dict['solved'][()],
            'done': self.rwd_dict['done'][()],
            'obs_dict': self.obs_dict,
            'rwd_dict': self.rwd_dict,
        }
        return env_info

    # compute vectorized rewards for paths
    def compute_path_rewards(self, paths):
        # path has two keys: observations and actions
        # path["observations"] : (num_traj, horizon, obs_dim)
        # path["rewards"] should have shape (num_traj, horizon)
        obs_dict = self.obsvec2obsdict(paths["observations"])
        rwd_dict = self.get_reward_dict(obs_dict)

        rewards = rwd_dict[RWD_MODE]
        done = rwd_dict['done']
        # time align rewards. last step is redundant
        done[...,:-1] = done[...,1:]
        rewards[...,:-1] = rewards[...,1:]
        paths["done"] = done if done.shape[0] > 1 else done.ravel()
        paths["rewards"] = rewards if rewards.shape[0] > 1 else rewards.ravel()
        return paths

    def truncate_paths(self, paths):
        hor = paths[0]['rewards'].shape[0]
        for path in paths:
            if path['done'][-1] == False:
                path['terminated'] = False
                terminated_idx = hor
            elif path['done'][0] == False:
                terminated_idx = sum(~path['done'])+1
                for key in path.keys():
                    path[key] = path[key][:terminated_idx+1, ...]
                path['terminated'] = True
        return paths

    def reset_model(self):
        self.np_random.choice(self.sims_idx) # To avoid some weird numpy behavior
        idx = self.np_random.choice(self.sims_idx)
        self.sim = self.sims[idx]
        self.data = self.sim.data
        self.model = self.sim.model

        self.sim.reset()
        self.model.site_pos[self.target_obj_sid,2] = self.np_random.uniform(low=0.1, high=0.25)
        self.sim.forward()
        return self.get_obs()

    def get_env_state(self):
        """
        Get state of hand as well as objects and targets in the scene
        """
        qpos = self.data.qpos.ravel().copy()
        qvel = self.data.qvel.ravel().copy()
        target_pos = self.data.site_xpos[self.target_obj_sid].ravel().copy()
        return dict(qpos=qpos, qvel=qvel, target_pos=target_pos)

    def set_env_state(self, state_dict):
        """
        Set the state which includes hand as well as objects and targets in the scene
        """
        qp = state_dict['qpos']
        qv = state_dict['qvel']
        self.set_state(qp, qv)
        self.sim.forward()

    def mj_viewer_setup(self):
        self.viewer = MjViewer(self.sim)
        self.viewer.cam.azimuth = 45
        self.viewer.cam.distance = 2.0
        self.sim.forward()

    # evaluate paths and log metrics to logger
    def evaluate_success(self, paths, logger=None):
        num_success = 0
        num_paths = len(paths)
        horizon = self.spec.max_episode_steps # paths could have early termination

        # success if door open for 5 steps
        for path in paths:
            if np.sum(path['env_infos']['solved']) > 5:
                num_success += 1
        success_percentage = num_success*100.0/num_paths

        # log stats
        if logger:
            rwd_sparse = np.mean([np.mean(p['env_infos']['rwd_sparse']) for p in paths]) # return rwd/step
            rwd_dense = np.mean([np.sum(p['env_infos']['rwd_dense'])/horizon for p in paths]) # return rwd/step
            logger.log_kv('rwd_sparse', rwd_sparse)
            logger.log_kv('rwd_dense', rwd_dense)
            logger.log_kv('success_rate', success_percentage)

        return success_percentage
