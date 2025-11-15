#!/usr/bin/env python3
"""
Script to load and run a trained tabular Q-learning model in the maze environment.
"""

import argparse
import pickle
from collections import defaultdict
from typing import Union

import numpy as np
import math as m
from maze_env import MazeEnv


def load_model(model_path: str) -> dict:
    """Load the trained Q-table model from a pickle file."""
    with open(model_path, "rb") as f:
        data = pickle.load(f)
    return data


def discretize_state(state: np.ndarray, cell: float = 0.10) -> tuple[int, int, int]:
    """
    Map continuous (x, y, yaw) to coarse discrete bins. Matches tabular Q-learning defaults.
    """
    x, y, yaw = float(state[0]), float(state[1]), float(state[2])
    i = int(round(x / cell))
    j = int(round(y / cell))
    yaw = yaw % (2.0 * m.pi)
    heading_bin = int(round(yaw / (m.pi / 2.0))) % 4
    return i, j, heading_bin


def greedy_action(
    q_table: Union[dict, defaultdict],
    state_key: tuple[int, int, int],
    action_dim: int,
    ) -> int:
    """Select the best action greedily from the Q-table (no exploration)."""
    if state_key not in q_table:
        # If state not in Q-table, default to action 0 (forward)
        print(f"Warning: State {state_key} not in Q-table, using action 0")
        return 0
    
    values = q_table[state_key]
    max_q = np.max(values)
    best_actions = np.flatnonzero(values == max_q)
    # If multiple best actions, pick the first one deterministically
    return int(best_actions[0])


def run_episode(
    env: MazeEnv,
    q_table: Union[dict, defaultdict],
    cell_size: float,
    max_steps: int = 1000,
    verbose: bool = True,
    ) -> dict:
    """Run a single episode using the trained Q-table policy."""
    state = env.reset()
    total_reward = 0.0
    steps = 0
    trajectory = [state.copy()]
    
    for step in range(max_steps):
        # Discretize current state
        state_disc = discretize_state(state, cell=cell_size)
        
        # Select action greedily
        action = greedy_action(q_table, state_disc, env.action_dim)
        
        # Take step in environment
        next_state, reward, done, info = env.step(action)
        
        total_reward += reward
        steps += 1
        trajectory.append(next_state.copy())
        
        if verbose and (step % 1 == 0 or done):
            print(
                f"Step {steps}: pos=({next_state[0]:.3f}, {next_state[1]:.3f}), "
                f"yaw={next_state[2]:.3f}, reward={reward:.2f}, "
                f"action={action}, collision={info.get('collision', False)}"
            )
        
        if done:
            if info.get("stage") == "goal" and env._goal_reached:
                if verbose:
                    print(f"✓ Goal reached in {steps} steps! Total reward: {total_reward:.2f}")
            else:
                if verbose:
                    print(f"Episode terminated at step {steps}. Total reward: {total_reward:.2f}")
            break
        
        state = next_state
    
    return {
        "steps": steps,
        "total_reward": total_reward,
        "success": env._goal_reached if hasattr(env, "_goal_reached") else False,
        "trajectory": trajectory,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Run a trained tabular Q-learning model in the maze environment."
    )
    parser.add_argument(
        "--model",
        type=str,
        default="models/q-tabular.pkl",
        help="Path to the trained Q-table model pickle file.",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Use PyBullet GUI rendering (visualize the robot).",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Number of episodes to run.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=1000,
        help="Maximum steps per episode.",
    )
    parser.add_argument(
        "--forward-distance",
        type=float,
        default=0.15,
        help="Per-step forward translation distance for the environment.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce output verbosity.",
    )
    args = parser.parse_args()
    
    # Load the trained model
    print(f"Loading model from {args.model}...")
    model_data = load_model(args.model)
    
    # Extract Q-table and parameters
    q_table_dict = model_data.get("q_table", {})
    # Convert back to defaultdict for easier access
    q_table = defaultdict(lambda: np.zeros(4, dtype=np.float32))
    q_table.update(q_table_dict)
    
    cell_size = model_data.get("cell_size", 0.10)
    training_params = model_data.get("training_params", {})
    
    print(f"Model loaded successfully!")
    print(f"  Cell size: {cell_size}")
    print(f"  Q-table entries: {len(q_table_dict)}")
    if training_params:
        print(f"  Training params: {training_params}")
    print()
    
    # Create environment
    env = MazeEnv(gui=args.gui, forward_distance=args.forward_distance)
    
    # Run episodes
    results = []
    for episode in range(args.episodes):
        if not args.quiet:
            print(f"=== Episode {episode + 1}/{args.episodes} ===")
        
        result = run_episode(
            env,
            q_table,
            cell_size,
            max_steps=args.max_steps,
            verbose=not args.quiet,
        )
        results.append(result)
        
        if not args.quiet:
            print()
    
    # Print summary
    print("=" * 50)
    print("Summary:")
    print(f"  Episodes run: {len(results)}")
    successes = sum(1 for r in results if r["success"])
    print(f"  Successful episodes: {successes}/{len(results)}")
    if results:
        avg_steps = np.mean([r["steps"] for r in results])
        avg_reward = np.mean([r["total_reward"] for r in results])
        print(f"  Average steps per episode: {avg_steps:.1f}")
        print(f"  Average reward per episode: {avg_reward:.2f}")
    
    env.close()


if __name__ == "__main__":
    main()

