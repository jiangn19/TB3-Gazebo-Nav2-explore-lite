# >>> TB3 initialize >>>
_turtle3init () {

    # 1. ROS 2 Humble
    source /opt/ros/humble/setup.bash

    # 2. Gazebo 环境（关键：让 Gazebo 知道插件/模型路径）
    if [ -f /usr/share/gazebo/setup.sh ]; then
        source /usr/share/gazebo/setup.sh
    elif [ -f /usr/share/gazebo/setup.bash ]; then
        source /usr/share/gazebo/setup.bash
    fi

    # 3. 自动检测并 source TurtleBot3 工作空间
    # 首先找到 vln_gazebo_simulator 目录
    _find_vln_simulator_dir() {
        local search_paths=(
            "."  # 当前目录
            ".."  # 父目录
            "../.."  # 祖父目录
            "../../.."  # 曾祖父目录
            "~/Documents"  # 默认工作空间
            "/home/chengsn/Workspace/VLN_ws"  # 用户特定路径
        )

        for path in "${search_paths[@]}"; do
            if [ -d "$path/vln_gazebo_simulator" ]; then
                echo "$path/vln_gazebo_simulator"
                return 0
            fi
        done
        return 1
    }

    # 获取 vln_gazebo_simulator 目录路径
    VLN_SIMULATOR_DIR=$(_find_vln_simulator_dir)
    if [ -n "$VLN_SIMULATOR_DIR" ]; then
        SETUP_BASH="$VLN_SIMULATOR_DIR/install/setup.bash"
        if [ -f "$SETUP_BASH" ]; then
            source "$SETUP_BASH"
            echo "Sourced workspace from: $VLN_SIMULATOR_DIR"
        else
            echo "Warning: Found vln_gazebo_simulator at $VLN_SIMULATOR_DIR but setup.bash not found"
        fi
    else
        echo "Warning: Could not find vln_gazebo_simulator directory"
    fi

    # 4. ROS / TB3 基本设置
    # export ROS_DOMAIN_ID=30
    export TURTLEBOT3_MODEL=waffle

    # 5. 自动构建 Gazebo 模型路径
    # 初始化 GAZEBO_MODEL_PATH
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:-}

    # 自动查找 TurtleBot3 模型路径
    _find_tb3_models() {
        local search_paths=(
            "$VLN_SIMULATOR_DIR/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models"
            "/opt/ros/humble/share/turtlebot3_gazebo/models"
        )

        for path in "${search_paths[@]}"; do
            if [ -d "$path" ]; then
                echo "$path"
                return 0
            fi
        done
        return 1
    }

    # 获取 TurtleBot3 模型路径
    TB3_MODELS=$(_find_tb3_models)
    if [ -n "$TB3_MODELS" ]; then
        export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$TB3_MODELS"
        echo "Added TurtleBot3 models: $TB3_MODELS"
    fi

    # 6. 用户自定义模型路径接口
    # 在这里添加你的自定义模型路径，例如：
    # CUSTOM_MODELS=(
        # "~/gazebo_envs/aws-robomaker-small-house-world/models"
        # "~/Documents/vln_gazebo_simulator/models"
        # "/path/to/your/custom/models"
    # )

    # 如果定义了自定义模型路径，就添加它们
    if [ ${#CUSTOM_MODELS[@]} -gt 0 ]; then
        for model_path in "${CUSTOM_MODELS[@]}"; do
            if [ -d "$model_path" ]; then
                export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$model_path"
                echo "Added custom models: $model_path"
            else
                echo "Warning: Custom model path not found: $model_path"
            fi
        done
    fi

    echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
}

alias turtle3init='_turtle3init'
# <<< TB3 initialize <<<

# 如果脚本被直接执行（而不是被 source），自动运行初始化
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Running TurtleBot3 environment setup..."
    _turtle3init
    echo "Setup complete. You can now use 'turtle3init' to reinitialize the environment."
fi