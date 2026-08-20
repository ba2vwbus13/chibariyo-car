FROM tiryoh/ros2-desktop-vnc:humble

SHELL ["/bin/bash", "-c"]

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

# ------------------------------------------------------------------
# 1) Gazebo Fortress 本体 + 確実に入るROSパッケージ
#    (ros_gz は arm64 にバイナリが無いことがあるので次のステージで扱う)
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    ignition-fortress \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cv-bridge \
    ros-humble-image-view \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    ffmpeg \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------
# 2) ros_gz (Gazebo <-> ROS 2 ブリッジ)
#    バイナリがあれば apt、無ければ humble ブランチをソースビルド
#    → arm64(Apple Silicon)でも通るようにフォールバックを用意
# ------------------------------------------------------------------
ENV GZ_VERSION=fortress
ENV IGNITION_VERSION=fortress

RUN set -ex; \
    apt-get update; \
    have() { apt-cache show "$1" 2>/dev/null | grep -q '^Version:'; }; \
    if have ros-humble-ros-gz; then \
        echo "=== ros_gz: apt (metapackage) ==="; \
        apt-get install -y --no-install-recommends ros-humble-ros-gz; \
    elif have ros-humble-ros-gz-sim; then \
        echo "=== ros_gz: apt (individual packages) ==="; \
        apt-get install -y --no-install-recommends \
            ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
            ros-humble-ros-gz-image ros-humble-ros-gz-interfaces; \
    elif have ros-humble-ros-ign; then \
        echo "=== ros_gz: apt (ros_ign 旧名) ==="; \
        apt-get install -y --no-install-recommends ros-humble-ros-ign; \
    else \
        echo "=== ros_gz: バイナリ無し → ソースビルド (数分〜十数分かかります) ==="; \
        mkdir -p /opt/ros_gz_ws/src; \
        git clone --depth 1 -b humble https://github.com/gazebosim/ros_gz.git /opt/ros_gz_ws/src/ros_gz; \
        rosdep init 2>/dev/null || true; \
        rosdep update --rosdistro humble; \
        rosdep install -y -r --from-paths /opt/ros_gz_ws/src -i --rosdistro humble; \
        set +u; source /opt/ros/humble/setup.bash; \
        cd /opt/ros_gz_ws; \
        export MAKEFLAGS="-j1"; \
        colcon build --merge-install \
            --packages-up-to ros_gz_sim ros_gz_bridge ros_gz_image \
            --executor sequential \
            --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release; \
        rm -rf /opt/ros_gz_ws/build /opt/ros_gz_ws/log; \
    fi; \
    rm -rf /var/lib/apt/lists/*

# YOLO(ultralytics) + NumPyバージョン固定
# (apt版OpenCVがNumPy 1.x向けのため "numpy<2" を同時指定して衝突回避)
RUN pip3 install --no-cache-dir ultralytics "numpy<2" && \
    pip3 install --no-cache-dir "setuptools==58.2.0"  # colcon --symlink-install互換のため固定

# ワークスペースの自動source(全ユーザー共通のbashrcに追記)
# 順番: ROS本体 → ros_gz(ソースビルド時のみ) → ユーザーのros2_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f /opt/ros_gz_ws/install/setup.bash ] && source /opt/ros_gz_ws/install/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
