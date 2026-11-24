# Notes for coding agents

kinex-sim runs public robotics simulators as managed ROS 2 graphs. Start with [README.md](README.md) and
[Design and usage](docs/README.md); each simulator has its own page under `docs/`.

- **Adding or porting a simulator:** follow [docs/adding-a-simulator.md](docs/adding-a-simulator.md).
  It covers the pinned source, the environment, package, manifest and task catalog, upstream's own
  episode controls, assets, devices under Kinex's contract, the backend, scoring parity with upstream's
  evaluator, the catalog rig, tests and docs.
- **Upstream first:** a simulator behaves and scores as its upstream does. Every departure is a
  patch in `patches/<sim>/`, documented on the simulator's page.
- **Rebuilds:** editing `pixi.toml` or `ros2/kinex_sim_runtime` marks every simulator's ROS build
  stale. Run `kinex-sim doctor <sim>`, then `kinex-sim prepare <sim>`; nothing downloads.
- **Checks:** [Verification](docs/README.md#verification) lists each simulator's tests, and
  [Kinex compatibility checks](docs/kinex.md#compatibility-checks) the ones that need Kinex.
