<div align="center">

  <img src="media/groot_wbc.png" width="800" alt="GEAR SONIC Header">

</div>

<div align="center">

[![License](https://img.shields.io/badge/License-Apache%202.0-76B900.svg)](LICENSE)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-2.3.0-orange.svg)](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.3.0)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-76B900.svg)](https://nvlabs.github.io/GR00T-WholeBodyControl/)

</div>

---

# GR00T-WholeBodyControl

这是 **GR00T Whole-Body Control (WBC)** 项目的代码仓库，包含人形机器人全身控制相关的模型权重、训练脚本、评测工具和部署代码。当前主要包含两条能力线：

- **Decoupled WBC**：用于 NVIDIA GR00T [N1.5](https://research.nvidia.com/labs/gear/gr00t-n1_5/) 和 [N1.6](https://research.nvidia.com/labs/gear/gr00t-n1_6/) 的上下身解耦控制器
- **GEAR-SONIC 系列**：最新一代通用人形机器人全身控制模型，详见 [白皮书](https://nvlabs.github.io/GEAR-SONIC/)

## 最新动态

- **[2026-03-16]** 开源 [BONES-SEED](https://huggingface.co/datasets/bones-studio/seed) 数据集，包含 142K+ 人体动作、约 288 小时数据，其中有大量兼容 Unitree G1 MuJoCo 的轨迹
- **[2026-02-19]** 发布 GEAR-SONIC：预训练策略、C++ 推理部署栈、VR 遥操作栈和完整文档
- **[2025-11-12]** 首次发布 GR00T-WholeBodyControl，包含用于 GR00T N1.5 / N1.6 的 Decoupled WBC

## GEAR-SONIC

<p style="font-size: 1.2em;">
    <a href="https://nvlabs.github.io/GEAR-SONIC/"><strong>Website</strong></a> |
    <a href="https://huggingface.co/nvidia/GEAR-SONIC"><strong>Model</strong></a> |
    <a href="https://arxiv.org/abs/2511.07820"><strong>Paper</strong></a> |
    <a href="https://nvlabs.github.io/GR00T-WholeBodyControl/"><strong>Docs</strong></a>
</p>

<div align="center">
  <img src="docs/source/_static/sonic-preview-gif-480P.gif" width="800">
</div>

SONIC 是一个人形机器人行为基础模型。它通过大规模人体动作数据学习核心运动技能，不再依赖为每个预定义动作单独训练控制器，而是把动作跟踪作为统一训练任务，让一个策略同时支持步行、爬行、遥操作和多模态控制等能力。

本仓库当前发布了 SONIC 的部署框架、预训练模型、遥操作栈以及部分配套数据与工具。

## VR 全身遥操作

SONIC 支持通过 PICO VR 头显进行实时全身遥操作，可用于自然的人到机器人动作迁移、交互控制和数据采集。

<div align="center">
<table>
<tr>
<td align="center"><b>Walking</b></td>
<td align="center"><b>Running</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_walking.gif" width="400"></td>
<td align="center"><img src="media/teleop_running.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Sideways Movement</b></td>
<td align="center"><b>Kneeling</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_sideways.gif" width="400"></td>
<td align="center"><img src="media/teleop_kneeling.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Getting Up</b></td>
<td align="center"><b>Jumping</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_getup.gif" width="400"></td>
<td align="center"><img src="media/teleop_jumping.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Bimanual Manipulation</b></td>
<td align="center"><b>Object Hand-off</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_bimanual.gif" width="400"></td>
<td align="center"><img src="media/teleop_switch_hands.gif" width="400"></td>
</tr>
</table>
</div>

## 运动学规划器

SONIC 还包含一个实时运动学规划器，可通过键盘或手柄切换运动风格，并在线调整速度、姿态高度等参数。

<div align="center">
<table>
<tr>
<td align="center" colspan="2"><b>In-the-Wild Navigation</b></td>
</tr>
<tr>
<td align="center" colspan="2"><img src="media/planner/planner_in_the_wild_navigation.gif" width="800"></td>
</tr>
<tr>
<td align="center"><b>Run</b></td>
<td align="center"><b>Happy</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_run.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_happy.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Stealth</b></td>
<td align="center"><b>Injured</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_stealth.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_injured.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Kneeling</b></td>
<td align="center"><b>Hand Crawling</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_kneeling.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_hand_crawling.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Elbow Crawling</b></td>
<td align="center"><b>Boxing</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_elbow_crawling.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_boxing.gif" width="400"></td>
</tr>
</table>
</div>

## 仓库内容

本次仓库主要包括：

- `gear_sonic_deploy`：C++ 推理与部署栈，可用于真机或 MuJoCo sim2sim
- `gear_sonic`：Python 侧仿真与遥操作栈
- `download_from_hf.py`：从 Hugging Face 拉取公开模型
- `install_scripts/`：MuJoCo、PICO、ROS 等安装脚本

## 快速开始

### 1. 克隆仓库并拉取大文件

这个仓库大量依赖 `git-lfs`。如果没有先拉取 LFS 文件，后续构建可能会把 `.a` 静态库当成文本指针文件，导致链接失败。

```bash
git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git
cd GR00T-WholeBodyControl
git lfs install
git lfs pull
```

### 2. 部署环境说明

推荐按官方的 **native / bare-metal** 方式部署：

- 桌面端系统：Ubuntu 22.04 或 24.04
- Python：3.10+ 即可
- GPU：NVIDIA 显卡
- 需要安装 CUDA Toolkit
- 需要安装 TensorRT

这条主部署路径 **不要求 conda**。  
`gear_sonic_deploy` 的主流程是系统依赖 + TensorRT + C++ 构建；MuJoCo 仿真则使用 `uv` 创建的 `.venv_sim`。

### 3. 配置 TensorRT

官方推荐桌面端使用 TensorRT 10.13 x86_64 的 TAR 包。解压后设置：

```bash
export TensorRT_ROOT="$HOME/TensorRT"
export PATH="$TensorRT_ROOT/bin:$PATH"
export LD_LIBRARY_PATH="$TensorRT_ROOT/lib:$LD_LIBRARY_PATH"
```

如果想持久化，可以把上面几行写入 `~/.bashrc`。

### 4. 安装部署依赖并构建

```bash
cd gear_sonic_deploy
chmod +x scripts/install_deps.sh
./scripts/install_deps.sh
source scripts/setup_env.sh
just build
```

构建成功后，主要二进制会出现在：

- `gear_sonic_deploy/target/release/g1_deploy_onnx_ref`
- `gear_sonic_deploy/target/release/freq_test`

### 5. 下载模型

如果模型文件还没准备好，可在仓库根目录运行：

```bash
python download_from_hf.py
```

默认会下载：

- `gear_sonic_deploy/policy/release/model_encoder.onnx`
- `gear_sonic_deploy/policy/release/model_decoder.onnx`
- `gear_sonic_deploy/policy/release/observation_config.yaml`
- `gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx`

该 Hugging Face 仓库目前是公开的，通常不需要 token。

## 最小可用测试

在正式跑 MuJoCo 之前，建议先确认部署栈至少能正常加载 ONNX 模型：

```bash
cd gear_sonic_deploy
source scripts/setup_env.sh
./target/release/freq_test policy/release/model_decoder.onnx 50 random
```

如果输出了输入输出维度、迭代时间和频率，说明 TensorRT / ONNX / 构建链路基本正常。

## MuJoCo sim2sim 测试

### 1. 安装 MuJoCo 仿真环境

在仓库根目录运行：

```bash
bash install_scripts/install_mujoco_sim.sh
```

这个脚本会：

- 安装 `uv`
- 创建 `.venv_sim`
- 安装 `gear_sonic[sim]`
- 安装 `unitree_sdk2_python`

注意：这里的 MuJoCo 流程**不是纯裸 MuJoCo**，而是通过 `unitree_sdk2py` 在仿真与部署程序之间做 DDS 桥接，所以 `unitree_sdk2_python` 是必需的。

### 2. 启动仿真

**Terminal 1：MuJoCo 仿真器**

```bash
cd /path/to/GR00T-WholeBodyControl
source .venv_sim/bin/activate
python gear_sonic/scripts/run_sim_loop.py
```

### 3. 启动部署程序

**Terminal 2：部署侧**

```bash
cd /path/to/GR00T-WholeBodyControl/gear_sonic_deploy
source ~/.bashrc
source scripts/setup_env.sh
bash deploy.sh sim
```

### 4. 运行时操作

#### 4.1 普通模式（参考动作跟踪）

普通模式会播放预加载的参考动作。程序启动后默认就是这个模式。

建议按下面顺序操作：

1. 在 Terminal 2 中按 `]` 启动控制系统
2. 点击 MuJoCo 窗口，按 `9` 将机器人放到地面
3. 回到 Terminal 2，按 `T` 播放当前参考动作，机器人会完整执行这一段动作
4. 按 `N` 切换到下一个动作序列，按 `P` 切换到上一个
5. 再按一次 `T` 播放新动作
6. 如果动作播完后想重播，继续按 `T`
7. 如果想在动作中途停止并回到第一帧，按 `R`，策略不会退出，机器人会停在当前动作的第一帧
8. 按 `Q` / `E` 可微调航向，每次约 `±π/12` 弧度
9. 按 `I` 可重新初始化基座四元数，并把当前朝向重置为参考动作的“正前方”
10. 完成后按 `O` 停止控制并退出

#### 4.2 规划器模式（实时动作生成）

规划器模式允许实时控制机器人。你可以切换动作风格、使用方向键控制移动，并在线调节速度与身体高度。

从普通模式切换到规划器模式：

1. 按 `ENTER` 切换到规划器模式，终端会打印 `Planner enabled`
2. 初始动作集为 `Locomotion`
3. 按 `1` / `2` / `3` 可分别切换为慢走、走路、奔跑

规划器模式常用按键如下：

- `W`：向前移动
- `S`：向后移动
- `A` / `D`：同时调整朝向和移动方向
- `Q` / `E`：原地转向，每次约 `±π/6` 弧度
- `,` / `.`：向左 / 向右平移
- `9` / `0`：减速 / 加速
- `N` / `P`：切换到下一个 / 上一个动作集
- `1`–`8`：在当前动作集中选择具体模式
- `-` / `=`：在下蹲类动作中降低 / 抬高身体高度，范围约为 `0.2–0.8 m`
- `R` / `` ` `` / `~`：立即清零运动动量，相当于紧急停下

补充说明：

- 机器人使用“动量”控制，按住方向键会把动量逐渐推到满值，松开后会缓慢减速回到静止
- `A` / `D` 更像“带转向的移动”，`Q` / `E` 则是“只改朝向、不改移动方向”
- 再按一次 `ENTER` 可回到普通模式
- 按 `O` 则直接停止控制并退出

### 5. 说明

- MuJoCo 仿真器运行在宿主机的 `.venv_sim` 中
- `gear_sonic_deploy` 可以运行在宿主机，也可以放在 Docker 容器中
- 如果使用 Docker，官方建议 Terminal 1 仍然放在宿主机，Terminal 2 再放进容器

## PICO VR 遥操作

### 1. 硬件与前置条件

PICO 全身遥操作通常需要：

- `PICO 4 / PICO 4 Pro` 头显
- 两个 PICO 手柄
- 两个 PICO 脚部运动追踪器
- 低延迟、稳定的同一局域网 Wi-Fi

开始之前，建议你先确认两件事：

- 已经跑通上面的 **MuJoCo sim2sim 测试**
- 已经安装并能启动 **XRoboToolkit PC Service**

在 `x86_64 + Ubuntu 22.04` 上，官方 PC 服务包通常是：

- `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`

安装后可以从系统应用菜单启动 `XRoboToolkit-PC-Service`，也可以直接运行：

```bash
/opt/apps/roboticsservice/run3D.sh
```

### 2. 在 PICO 头显中安装 XRoboToolkit App

在头显内完成以下步骤：

1. 先完成 PICO 的首次开机配置，并连接 Wi-Fi
2. 打开浏览器，访问 <https://github.com/XR-Robotics>
3. 开启开发者模式
4. 在头显里下载 `XRoboToolkit-PICO-1.1.1.apk`
5. 安装后，应用会出现在 Unknown / 未知来源 应用列表中

### 3. 配置脚部追踪器

1. 左右脚踝各绑一个追踪器，指示灯朝上
2. 尽量穿紧身裤或贴身运动裤，避免宽松裤腿遮挡追踪器
3. 在 PICO 开发者设置中关闭 `Safeguard`
4. 进入运动追踪器配对界面，先取消已有配对，再重新配对
5. 长按追踪器顶部按钮约 `6` 秒进入配对模式

校准时建议：

1. 先戴好头显
2. 按蓝色 `Calibrate`
3. 第一步站直、双臂自然下垂
4. 第二步低头看向脚部追踪器，让头显识别到它们

### 4. 安装 PICO Teleop Python 环境

在仓库根目录运行：

```bash
bash install_scripts/install_pico.sh
```

这个脚本会创建 `.venv_teleop`，其中包含：

- `gear_sonic[teleop]`
- `gear_sonic[sim]`
- `XRoboToolkit SDK`
- `unitree_sdk2_python`

激活方式：

```bash
source .venv_teleop/bin/activate
```

### 5. 连接 PICO 与 PC Service

1. 确保电脑和 PICO 在同一个 Wi-Fi 下
2. 在电脑上查本机局域网 IP，例如：

```bash
hostname -I
```

3. 打开头显中的 `XRoboToolkit`
4. 在 `PC Service` 一栏输入电脑 IP
5. 如果连接正常，状态会显示 `WORKING`

建议在 XRoboToolkit 中勾选：

- `Tracking` 下的 `Head`
- `Tracking` 下的 `Controller`
- `Data/Control` 下的 `Send`
- `Pico Motion Tracker` 下的 `Full body`

### 6. 在仿真中运行 PICO 遥操作

先确保 `XRoboToolkit PC Service` 已经在电脑上运行，然后开启三个终端。

**Terminal 1：MuJoCo 仿真器**

```bash
cd /path/to/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py
```

**Terminal 2：C++ 部署侧**

```bash
cd /path/to/GR00T-WholeBodyControl/gear_sonic_deploy
source ~/.bashrc
source scripts/setup_env.sh
bash deploy.sh sim --input-type zmq_manager

 ./deploy.sh enp5s0 --input-type zmq_manager real 

```

等待终端里出现 `Init done`。

**Terminal 3：PICO 数据流与可视化**

```bash
cd /path/to/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager \
    --vis_vr3pt --vis_smpl
```

如果只想跑最简模式，也可以：

```bash
python gear_sonic/scripts/pico_manager_thread_server.py --manager
```

### 7. 第一次遥操作怎么做

建议按下面顺序来：

1. 先站成校准姿势：站直、双脚并拢、上臂贴身下垂、前臂向前弯曲约 `90°`
2. 在 PICO 手柄上同时按 `A+B+X+Y`，启动策略并执行首次完整校准
3. 观察机器人当前姿态，把自己的手臂姿态尽量对齐
4. 同时按 `A+X`，切换到 `POSE` 全身遥操作模式
5. 这时你的全身动作就会映射到机器人
6. 再按一次 `A+X`，可以退回 `PLANNER` 模式
7. 结束或急停时，再按一次 `A+B+X+Y`

### 8. 常用按键

- `A+B+X+Y`：启动 / 停止策略；第一次按下时会触发完整校准
- `A+X`：在 `PLANNER` 和 `POSE` 间切换
- `B+Y`：切到 `PLANNER_FROZEN_UPPER`
- 左摇杆按下：在 Planner 模式与 `VR_3PT` 之间切换，并触发手腕校准
- 左摇杆：控制移动方向
- 右摇杆横向：控制航向
- `A+B`：切换到下一个 locomotion 模式
- `X+Y`：切换到上一个 locomotion 模式
- 手柄扳机：控制对应手部抓取

### 9. 使用建议与安全提示

- 真机前一定先在 MuJoCo 里练熟整套流程
- 切到 `POSE` 或 `VR_3PT` 之前，先让自己的姿态尽量接近机器人当前姿态
- 一定要穿不遮挡脚踝追踪器的裤子
- 网络延迟越低越好，PICO 和电脑最好在同一局域网下
- 保持至少约 `3 m` 安全区域，并始终有人在键盘旁准备按 `O`

如果只是做仿真练习，这条命令链路通常最关键：

- Terminal 1：`run_sim_loop.py`
- Terminal 2：`bash deploy.sh sim --input-type zmq_manager`
- Terminal 3：`pico_manager_thread_server.py --manager`

## 常见问题

### 1. `uv installation succeeded but binary not found on PATH`

有些系统，尤其是通过 VS Code Snap 环境触发安装时，`uv` 可能会被装到：

```bash
~/snap/code/current/.local/bin/uv
```

但安装脚本只会回退检查 `~/.local/bin` 或 `~/.cargo/bin`。  
这种情况下可以手动创建软链接：

```bash
mkdir -p "$HOME/.local/bin"
ln -sf "$HOME/snap/code/current/.local/bin/uv" "$HOME/.local/bin/uv"
ln -sf "$HOME/snap/code/current/.local/bin/uvx" "$HOME/.local/bin/uvx"
```

然后重新运行对应脚本，例如：

```bash
bash install_scripts/install_mujoco_sim.sh
bash install_scripts/install_pico.sh
```

### 2. `/usr/bin/ld: ... libunitree_sdk2.a:1: syntax error`

这通常不是代码错误，而是 `git-lfs` 文件没有拉全。  
如果 `gear_sonic_deploy/thirdparty/unitree_sdk2/lib/x86_64/libunitree_sdk2.a` 只有一百多字节、内容看起来像下面这样：

```text
version https://git-lfs.github.com/spec/v1
oid sha256:...
size ...
```

说明它仍然是 LFS 指针文件，而不是真实静态库。修复方式：

```bash
git lfs install
git lfs pull
```

然后重新构建：

```bash
cd gear_sonic_deploy
just build
```

### 3. 是否必须在 conda 环境里运行

不是。

- `gear_sonic_deploy`：推荐系统环境直接构建
- MuJoCo：使用 `.venv_sim`
- PICO VR：使用 `.venv_teleop`

除非你明确要走 RoboStack / ROS 的那条 conda 路线，否则主部署流程不建议强行塞进 conda。

### 4. 如何检查 XRoboToolkit PC Service 是否已经启动

如果已经安装了 PC Service，可以这样检查：

```bash
pgrep -a -f RoboticsServiceProcess
```

如果服务已经起来，通常会看到类似：

```text
/opt/apps/roboticsservice/RoboticsServiceProcess
```

也可以直接手动启动：

```bash
/opt/apps/roboticsservice/run3D.sh
```

## 文档

完整文档：<https://nvlabs.github.io/GR00T-WholeBodyControl/>

### 入门

- [Installation Guide](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html)
- [Quick Start](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/quickstart.html)
- [VR Teleoperation Setup](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/vr_teleop_setup.html)

### 教程

- [Keyboard Control](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/keyboard.html)
- [Gamepad Control](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/gamepad.html)
- [ZMQ Communication](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/zmq.html)
- [VR Whole-Body Teleop](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vr_wholebody_teleop.html)

### 进阶参考

- [Teleoperation Guide](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/teleoperation.html)
- [Decoupled WBC 文档](docs/source/references/decoupled_wbc.md)

---

## Citation

如果你在研究中使用了 GEAR-SONIC，请引用：

```bibtex
@article{luo2025sonic,
    title={SONIC: Supersizing Motion Tracking for Natural Humanoid Whole-Body Control},
    author={Luo, Zhengyi and Yuan, Ye and Wang, Tingwu and Li, Chenran and Chen, Sirui and Casta\~neda, Fernando and Cao, Zi-Ang and Li, Jiefeng and Minor, David and Ben, Qingwei and Da, Xingye and Ding, Runyu and Hogg, Cyrus and Song, Lina and Lim, Edy and Jeong, Eugene and He, Tairan and Xue, Haoru and Xiao, Wenli and Wang, Zi and Yuen, Simon and Kautz, Jan and Chang, Yan and Iqbal, Umar and Fan, Linxi and Zhu, Yuke},
    journal={arXiv preprint arXiv:2511.07820},
    year={2025}
}
```

---

## License

本项目采用双许可证：

- **源码**：Apache License 2.0
- **模型权重**：NVIDIA Open Model License

完整文本见 [LICENSE](LICENSE) 和 `legal/` 目录。使用前请同时确认源码许可证与模型许可证的要求。

---

## Support

如果你有问题或想反馈，请联系 GEAR WBC 团队：<gear-wbc@nvidia.com>

## TODO

- [x] 发布 SONIC 预训练策略权重
- [x] 开源 C++ 推理部署栈
- [x] 发布安装与使用文档
- [x] 开源遥操作栈和演示脚本
- [ ] 发布动作模仿 / 微调训练脚本
- [ ] 开源大规模数据采集工作流与 VLA 微调脚本
- [ ] 发布更多预处理动作数据

## 致谢

本仓库部分代码和设计参考了以下项目：

- [Beyond Mimic](https://github.com/HybridRobotics/whole_body_tracking)
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
