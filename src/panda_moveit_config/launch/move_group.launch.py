from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 🔥 Inject sim time directly into parameters
    moveit_config.robot_description["use_sim_time"] = True

    return generate_move_group_launch(moveit_config)