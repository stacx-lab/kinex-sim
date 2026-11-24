"""BiDex-100 control acceptance: both arms, a right-hand curl and an RGB-D head-camera frame."""
from kinex_sim_runtime import dual_arm_demo

# Index-finger MCP flexion, PIP and DIP (joints 5, 7 and 8) in the Sharpa hand's thumb-first order.
CURL = {5: .6, 7: .6, 8: .6}


def main():
    dual_arm_demo.main("bidex_control_demo", "head_camera", depth=True, hand=CURL)


if __name__ == "__main__":
    main()
