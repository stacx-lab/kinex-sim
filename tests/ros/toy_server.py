"""Run the shared ROS node against the synthetic backend in a separate process."""
from rosbridge_runtime import sim_node
from tests.toy_backend import PoseToy, ToyBackend

sim_node.load_backend = lambda name: PoseToy if name == "pose_toy" else ToyBackend
sim_node.main()
