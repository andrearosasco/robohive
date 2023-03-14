""" =================================================
Copyright (C) 2018 Vikash Kumar
Author  :: Vikash Kumar (vikashplus@gmail.com), Sudeep Dasari (sdasari@andrew.cmu.edu )
Source  :: https://github.com/vikashplus/mj_envs
License :: Under Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
================================================= """

import gym
from mj_envs.envs import env_base
from mj_envs.envs.tcdm.reference_motion import ReferenceMotion
from mj_envs.utils.quat_math import quat2euler, euler2quat, quatDiff2Vel
import numpy as np
import os
import collections

# ToDo
# - change target to reference


class TrackEnv(env_base.MujocoEnv):

    DEFAULT_CREDIT = """\
    Learning Dexterous Manipulation from Exemplar Object Trajectories and Pre-Grasps
        Sudeep Dasari, Abhinav Gupta, Vikash Kumar
        ICRA-2023 | https://pregrasps.github.io
    """

    DEFAULT_OBS_KEYS = [
        'qp', 'qv', 'hand_qpos_err','hand_qvel_err','obj_com_err'
    ]
    DEFAULT_RWD_KEYS_AND_WEIGHTS = {
        "pose": 0.0,#1.0,
        "object": 1.0,
        "bonus": 1.0,
        "penalty": -2,
    }
    def __init__(self, object_name, model_path, obsd_model_path=None, seed=None, **kwargs):

        print(self.DEFAULT_CREDIT)
        curr_dir = os.path.dirname(os.path.abspath(__file__))

        # Process model_path to import the right object
        with open(curr_dir+model_path, 'r') as file:
            processed_xml = file.read()
            processed_xml = processed_xml.replace('OBJECT_NAME', object_name)
        processed_model_path = curr_dir+model_path[:-4]+"_processed.xml"
        with open(processed_model_path, 'w') as file:
            file.write(processed_xml)
        self._object_name = object_name
        # Process obsd_model_path to import the right object
        if obsd_model_path == model_path:
            processed_obsd_model_path = processed_model_path
        elif obsd_model_path:
            with open(curr_dir+obsd_model_path, 'r') as file:
                processed_xml = file.read()
                processed_xml = processed_xml.replace('OBJECT_NAME', object_name)
            processed_obsd_model_path = curr_dir+model_path[:-4]+"_processed.xml"
            with open(processed_obsd_model_path, 'w') as file:
                file.write(processed_xml)
        else:
            processed_obsd_model_path = None

        # EzPickle.__init__(**locals()) is capturing the input dictionary of the init method of this class.
        # In order to successfully capture all arguments we need to call gym.utils.EzPickle.__init__(**locals())
        # at the leaf level, when we do inheritance like we do here.
        # kwargs is needed at the top level to account for injection of __class__ keyword.
        # Also see: https://github.com/openai/gym/pull/1497
        gym.utils.EzPickle.__init__(self, object_name, model_path, obsd_model_path, seed, **kwargs)

        # This two step construction is required for pickling to work correctly. All arguments to all __init__
        # calls must be pickle friendly. Things like sim / sim_obsd are NOT pickle friendly. Therefore we
        # first construct the inheritance chain, which is just __init__ calls all the way down, with env_base
        # creating the sim / sim_obsd instances. Next we run through "setup"  which relies on sim / sim_obsd
        # created in __init__ to complete the setup.
        super().__init__(model_path=processed_model_path, obsd_model_path=processed_obsd_model_path, seed=seed)
        os.remove(processed_model_path)
        if processed_obsd_model_path and processed_obsd_model_path!=processed_model_path:
            os.remove(processed_obsd_model_path)

        ##########################################
        self.lift_bonus_thresh =  0.02
        ### PRE-GRASP
        self.obj_err_scale = 50
        self.base_err_scale = 40
        self.lift_bonus_mag = 1 #2.5

        ### DEEPMIMIC
        self.qpos_reward_weight = 0.35
        self.qpos_err_scale = 5.0

        self.qvel_reward_weight = 0.05
        self.qvel_err_scale = 0.1

        # TERMINATIONS FOR OBJ TRACK
        self.obj_com_term = 0.5
        # TERMINATIONS FOR HAND-OBJ DISTANCE
        self.base_fail_thresh = .25
        self.TermObj = True

        # TERMINATIONS FOR MIMIC
        self.qpos_fail_thresh = .75
        self.TermPose = False
        ##########################################

        self._lift_z = self.sim.data.get_body_xipos(self._object_name)[2] + self.lift_bonus_thresh
        self.initialized_pos = False
        self._setup(**kwargs)


    def _setup(self,
               reference,                       # reference target/motion for behaviors
               motion_start_time:float=0,       # useful to skip initial motion
               motion_extrapolation:bool=True,  # Hold the last frame if motion is over
               obs_keys=DEFAULT_OBS_KEYS,
               weighted_reward_keys=DEFAULT_RWD_KEYS_AND_WEIGHTS,
               **kwargs):

        # prep reference
        self.ref = ReferenceMotion(reference=reference, motion_extrapolation=motion_extrapolation)
        self.motion_start_time = motion_start_time
        self.target_sid = self.sim.model.site_name2id("target")

        super()._setup(obs_keys=obs_keys,
                       weighted_reward_keys=weighted_reward_keys,
                       frame_skip=10,
                       **kwargs)
        # Adjust horizon if not motion_extrapolation
        if motion_extrapolation == False:
            self.spec.max_episode_steps = self.ref.horizon # doesn't work always. WIP
        # Adjust init as per the specified key
        ref0 = self.ref.get_reference(self.motion_start_time)
        self.init_qpos[:self.ref.robot_dim] = ref0.robot
        self.init_qpos[self.ref.robot_dim:self.ref.robot_dim+3] = ref0.object[:3]
        self.init_qpos[-3:] = quat2euler(ref0.object[3:])

        # hack because in the super()._setup the initial posture is set to the average qpos and when a step is called, it ends in a `done` state
        self.initialized_pos = True
        # if self.sim.model.nkey>0:
            # self.init_qpos[:] = self.sim.model.key_qpos[0,:]


    def rotation_distance(self, q1, q2, euler=True):
        if euler:
            q1 = euler2quat(q1)
            q2 = euler2quat(q2)
        return np.abs(quatDiff2Vel(q2,q1,1))


    def update_reference_insim(self, curr_ref):
        if curr_ref.object is not None:
            # print(curr_ref.object)
            self.sim.model.site_pos[:] = curr_ref.object[:3]
            self.sim_obsd.model.site_pos[:] = curr_ref.object[:3]
            self.sim.forward()

    def norm2(self, x):
        return np.sum(np.square(x))

    def get_obs_dict(self, sim):
        obs_dict = {}

        # get reference for current time (returns a named tuple)
        curr_ref = self.ref.get_reference(sim.data.time+self.motion_start_time)
        self.update_reference_insim(curr_ref)

        obs_dict['time'] = np.array([self.sim.data.time])
        obs_dict['qp'] = sim.data.qpos.copy()
        obs_dict['qv'] = sim.data.qvel.copy()
        obs_dict['robot_err'] = obs_dict['qp'][:30].copy() - curr_ref.robot

        ## info about current hand pose + vel
        obs_dict['curr_hand_qpos'] = sim.data.qpos[:-6].copy() ## assuming only 1 object and the last values are posision + rotation
        obs_dict['curr_hand_qvel'] = sim.data.qvel[:-6].copy() ## not used for now

        ## info about target hand pose + vel
        obs_dict['targ_hand_qpos'] = curr_ref.robot
        # import ipdb; ipdb.set_trace()
        obs_dict['targ_hand_qvel'] = curr_ref.robot_vel

        ## info about current object com + rotations
        obs_dict['curr_obj_com'] = sim.data.get_body_xipos(self._object_name).copy()
        obs_dict['curr_obj_rot'] = sim.data.get_body_xquat(self._object_name).copy()

        obs_dict['wrist_err'] = sim.data.get_body_xipos('wrist')

        obs_dict['base_error'] = obs_dict['curr_obj_com'] - obs_dict['wrist_err']

        ## info about target object com + rotations
        obs_dict['targ_obj_com']  = curr_ref.object[:3]
        obs_dict['targ_obj_rot']  = curr_ref.object[3:]

        ## Errors
        obs_dict['hand_qpos_err'] = obs_dict['curr_hand_qpos']-obs_dict['targ_hand_qpos']
        obs_dict['hand_qvel_err'] = obs_dict['curr_hand_qvel']-obs_dict['targ_hand_qvel']

        obs_dict['obj_com_err'] =  obs_dict['curr_obj_com'] - obs_dict['targ_obj_com']

        # self.sim.model.body_names --> body names
        return obs_dict


    def get_reward_dict(self, obs_dict):
        # get targets from reference object
        tgt_obj_com = obs_dict['targ_obj_com'].flatten()
        tgt_obj_rot = obs_dict['targ_obj_rot'].flatten()

        # get real values from physics object
        obj_com = obs_dict['curr_obj_com'].flatten()
        obj_rot = obs_dict['curr_obj_rot'].flatten()

        # calculate both object "matching"
        obj_com_err = np.sqrt(self.norm2(tgt_obj_com - obj_com))
        obj_rot_err = self.rotation_distance(obj_rot, tgt_obj_rot, False) / np.pi
        obj_reward = np.exp(-self.obj_err_scale * (obj_com_err + 0.1 * obj_rot_err))

        # calculate lift bonus
        lift_bonus = (tgt_obj_com[2] >= self._lift_z) and (obj_com[2] >= self._lift_z)

        # reward = obj_reward + self.lift_bonus_mag * float(lift_bonus)

        # calculate reward terms
        qpos_reward = np.exp(-self.qpos_err_scale * self.norm2(obs_dict['hand_qpos_err']))
        qvel_reward = np.exp(-self.qvel_err_scale * self.norm2(obs_dict['hand_qvel_err']))


        # weight and sum individual reward terms
        pose_reward  = self.qpos_reward_weight * qpos_reward
        vel_reward   = self.qvel_reward_weight * qvel_reward

        # print(f"Time: {obs_dict['time']} Error Pose: {self.norm2(obs_dict['hand_qpos_err'])} {obs_dict['hand_qpos_err']}    Error Obj:{obs_dict['obj_com_err']}")

        base_error = np.sqrt(self.norm2(obs_dict['base_error'] ))
        base_reward = np.exp(-self.base_err_scale * base_error)

        # print(base_error, base_reward)

        rwd_dict = collections.OrderedDict((
            # Optional Keys
            ('pose',   float(pose_reward+vel_reward)),
            ('object' , float(obj_reward+base_reward)),
            ('bonus',  self.lift_bonus_mag * float(lift_bonus)),
            ('penalty', float(self.check_termination(obs_dict))),
            # Must keys
            ('sparse',  0),
            ('solved',  0),
            ('done',    self.initialized_pos and self.check_termination(obs_dict)),
            # ('done',   False),
        ))
        rwd_dict['dense'] = np.sum([wt*rwd_dict[key] for key, wt in self.rwd_keys_wt.items()], axis=0)

        # print(rwd_dict['dense'], obj_com_err,rwd_dict['done'],rwd_dict['sparse'])
        return rwd_dict

    def playback(self):
        idxs = self.ref.find_timeslot_in_reference(self.time)
        # print(f"Time {self.time} {idxs} {self.ref.horizon}")
        ref_mot = self.ref.get_reference(self.time)
        rob_mot = ref_mot.robot
        obj_mot = ref_mot.object
        # import ipdb; ipdb.set_trace()
        # objt = self.ref.ref_file['object_translation'][int(self.ref.ref_file['grasp_frame'])+idxs[0],:]
        # print(idxs,np.sum(objt - obj_mot[:3]),objt,obj_mot[:3])
        self.sim.data.qpos[:len(rob_mot)] = rob_mot
        self.sim.data.qpos[len(rob_mot):len(rob_mot)+3] = obj_mot[:3]

        self.sim.data.qpos[len(rob_mot)+3:] = quat2euler(obj_mot[3:])
        self.sim.forward()
        self.sim.data.time = self.sim.data.time + 0.02#self.env.env.dt

        return idxs[0] < self.ref.horizon-1

    def reset(self):
        # print("Reset")
        self.ref.reset()
        obs = super().reset(self.init_qpos, self.init_qvel)
        # print(self.time, self.sim.data.qpos)
        return obs


    def check_termination(self, obs_dict):

        if self.TermObj: # termination on object
            obj_term = self.norm2(obs_dict['obj_com_err']) >= self.obj_com_term ** 2
            base_term =self.norm2(obs_dict['base_error'] ) >= self.base_fail_thresh ** 2
        else:
            obj_term, base_term = False, False

        if self.TermPose: # termination on posture
            qpos_term = self.norm2(obs_dict['hand_qpos_err']) >= self.qpos_fail_thresh
        else:
            qpos_term = False

        return obj_term or qpos_term or base_term # combining termination for object + posture