FROM tiryoh/ros2-desktop-vnc:humble

# ROSリポジトリのGPG鍵を最新に更新(鍵ローテーションによるEXPKEYSIGエラー対策)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# OSRF(Gazebo)リポジトリを追加
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# ダウンロード失敗時の自動リトライ設定
RUN printf 'Acquire::Retries "10";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n' \
    > /etc/apt/apt.conf.d/80-retries

# Gazebo Fortress (arm64/amd64両対応) とROS連携パッケージ
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-cv-bridge \
    ros-humble-image-view \
    python3-opencv \
    python3-colcon-common-extensions \
    ffmpeg \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

# YOLO(ultralytics) + NumPyバージョン固定
# (apt版OpenCVがNumPy 1.x向けのため "numpy<2" を同時指定して衝突回避)
RUN pip3 install --no-cache-dir ultralytics "numpy<2" && \
    pip3 install --no-cache-dir "setuptools==58.2.0"  # colcon --symlink-install互換のため固定

# ワークスペースの自動source(全ユーザー共通のbashrcに追記)
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
