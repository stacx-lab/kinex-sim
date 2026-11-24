"""Which evaluator episode a launch seed names, and BiDex-100's camera names; imports neither MuJoCo nor ROS."""

EPISODES, OOD_SEED = 25, 2  # the official evaluator's episodes per eval seed and first OOD eval seed
CAMERAS = {"head_camera": "head", "left_camera": "left_wrist", "right_camera": "right_wrist"}


def episode(seed, ood):
    """The official evaluator's (eval seed, episode) for a seed: 0-49 replay its rollouts."""
    eval_seed, index = divmod(int(seed), EPISODES)
    return eval_seed + OOD_SEED * ood, index
