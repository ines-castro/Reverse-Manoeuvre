import optuna
import yaml
import numpy as np
from control import Controller
from geometry_solver import GeometrySolver

with open('configs/overshoot.yaml', 'r') as f:
    configs = yaml.safe_load(f)

def objective(trial):

    parameters = {
        'physics': {
            'dt': 0.1
        },
        'controller': {
            'constant_vx': -0.1, 
            'K_dist': trial.suggest_float("K_dist", 0.1, 50.0), 
            'K_turn': trial.suggest_float("K_turn", 0.01, 10.0), 
            'K_hitch': trial.suggest_float("K_hitch", 0.1, 50.0), 
            'lookahead_distance': 1, #trial.suggest_float("lookahead_distance", 0.1, 2.0), 
            'horizon': 10, #trial.suggest_int("horizon", 5, 40)
        },
        'cart_dimensions': {
            'width': configs['cart_dimensions']['width'], 
            'fixed_wheel_dist': configs['cart_dimensions']['fixed_wheel_dist'], 
            'gripper_length': configs['cart_dimensions']['gripper_length'], 
            'gripper_angle_limit': configs['cart_dimensions']['gripper_angle_limit']
        }
    }
    
    # Initial setup
    raw_state = configs['physics']['initial_state']
    state = np.array([raw_state[0], raw_state[1], np.deg2rad(raw_state[2]), np.deg2rad(raw_state[3])])  
    target = configs['physics']['target']
    solver = GeometrySolver(state, target, turning_radius=configs['physics']['turning_radius'])
    ctrl = Controller(state, target, solver, parameters)

    evaluation_score = 0
    for step in range(1000):
        state, _, _, _, e_cross, finished = ctrl.path_following(state, target)
        
        # Penalize cross-track error and excessive hitching
        evaluation_score += e_cross**2 + (abs(state[3]) * 2)
        
        # Stop if it reaches the target
        if finished: break

        # Only fail if it's really far away or completely jackknifed
        if e_cross > 10.0 or abs(np.rad2deg(state[3])) > (configs['cart_dimensions']['gripper_angle_limit'] + 5):
            return 50000000
    
    # It has to reach the target
    if not finished: 
        print(" ✖ This trial did not reach the target")
        return 1000000000

    return evaluation_score

study = optuna.create_study(study_name='controller-tunning')

# graphiic display for hyperparameter importance
study.optimize(objective, n_trials=800)

print("--- Best parameters ---")
for key, value in study.best_params.items():
    print(f"{key}: {value}")