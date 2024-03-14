# Meta-Bandit-Sequential-Greedy

## About
This repository holds the implementation codes for the simulation scenario in the paper.

Zirui Xu*, Xiaofeng Lin*, and Vasileios Tzoumas, ["Leveraging Untrustworthy Commands for Multi-Robot Coordination in
Unpredictable Environments: A Bandit Submodular Maximization Approach"](https://arxiv.org/abs/2309.16161), American Control Conference 2024.

This repository base on the implementation codes of [Bandit Submodular Maximization for Multi-Robot Coordination in Unpredictable and Partially Observable Environments](https://arxiv.org/abs/2305.12795). The github repo can be found [here](https://github.com/UM-iRaL/bandit-sequential-greedy).

This repository contains the codes for several scenarios that vary in the number of agents/targets, initial pose of agents/targets and accuracy of predictions.

## Run the simulation
run ```main.m```

### Design new scenarios

To change the number of robots/the number of targets/the type of a target/base learner, please modify the following parameters in ```main.m```:
```matlab
    num_robot     % number of robots
    num_tg        % number of targets
    type_tg       % type of targets ("normal" or "adversarial")
    base_learner  % options: human/greedy
```

To modify settings of robots, targets and external commands, please change the following parameters in ```scenarios_settings.m``` (notice all variables should have matching dimensions):
```matlab
    v_robot       % speed of robots
    r_senses      % sensing range of robots
    fovs          % field of view in degree
    v_tg          % speed of targets
    yaw_tg        % initial yaw angles of targets
    motion_tg     % type of motion of targets (circle, straight)
    x_true_init   % initial pose of robots
    tg_true_init  % initial pose of targets
    human_pred    % external/untrusty commands
```
## License
The project is licensed under MIT License.

## Citation
If you have an academic use, please cite:

```
@misc{xu2023leveraging,
      title={Leveraging Untrustworthy Commands for Multi-Robot Coordination in Unpredictable Environments: A Bandit Submodular Maximization Approach}, 
      author={Zirui Xu and Xiaofeng Lin and Vasileios Tzoumas},
      year={2023},
      eprint={2309.16161},
      archivePrefix={arXiv},
      primaryClass={eess.SY}
}
```