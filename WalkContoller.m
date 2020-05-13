import org.opensim.modeling.*;
% pyversion('C:\Users\csy\AppData\Local\conda\conda\envs\matlab-rl\python.exe')


mode_ = '2D';
difficulty = 3;
visualize=true;
seed=[];
sim_dt = 0.01;
sim_t = 10;
timstep_limit = fix(sim_t/sim_dt);


INIT_POSE = [
    1.7,... % forward speed
    .0,... % rightward speed   
    9.023245653983965608e-01,... % pelvis height
    2.012303881285582852e-01,... % trunk lean 
    0*pi/180,... % [right] hip adduct
    -6.952390849304798115e-01,... % hip flex
    -3.231075259785813891e-01,... % knee extend
    1.709011708233401095e-01,... % ankle flex
    0*pi/180,... % [left] hip adduct
    -5.282323914341899296e-02,... % hip flex
    -8.041966456860847323e-01,... % knee extend
    -1.745329251994329478e-01]; % ankle flex

if strcmp(mode_,'2D')
    params = load('params_2D.txt');
elseif strcmp(mode_,'3D')
    params = load('params_3D_init.txt');
end

locoCtrl = OsimReflexCtrl(mode_, sim_dt);
env = L2M2019Env(visualize, 5e-5, difficulty, seed, []);
env = env.change_model(mode_, difficulty, seed);
[env, obs_dict] = env.reset(true, seed, true, INIT_POSE);
env.spec.timestep_limit = timstep_limit;

total_reward = 0;
t = 0;
i = 0;
while true
    i = i + 1;
    t = t + sim_dt;

    locoCtrl = locoCtrl.set_control_params(params);
    [locoCtrl, action] = locoCtrl.update(obs_dict);
    [env, obs_dict, reward, done, info] = env.step(action, true, true);
    total_reward = total_reward + reward;
    fprintf('Time is %3.2f  Reward is %3.3f\n', t, reward)
    fprintf('Action3 is %2.2f\n', action(3))
    if done
        break
    end
end

print('    score=%6.2f time=%6.2f sec\n',total_reward, t)








