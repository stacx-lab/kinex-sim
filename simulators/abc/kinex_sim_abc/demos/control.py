"""ABC control acceptance: both YAM arms, a gripper cycle and an RGB-D top-camera frame."""
from kinex_sim_runtime import dual_arm_demo


def main():
    dual_arm_demo.main("abc_control_demo", "camera_top")


if __name__ == "__main__":
    main()
