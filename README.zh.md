# motor_test

这个目录包含一组用于测试 LeRobot/Feetech 电机通信、扫描、电机单独控制，以及夹爪开合的 Python 脚本。

英文说明请见 [README.md](/Users/jonathanlehner/wundercode/robotics/motor_test/README.md)。

## 用 bash 脚本快速开始

最简单的环境准备方式是：

```bash
./setup_env.sh
```

这个命令会：

- 在不存在时创建 `.venv`
- 在脚本内部激活虚拟环境
- 优先尝试中国大陆可访问的镜像源
- 升级 `pip`
- 安装 `requirements.txt` 中的依赖

如果你希望当前终端会话保持激活状态，请使用：

```bash
source ./setup_env.sh
```

之后就可以直接用 `python ...` 运行脚本。

脚本默认按下面的顺序尝试软件源：

- 清华 Tuna 镜像
- 阿里云 PyPI 镜像
- 官方 PyPI 作为最后兜底

如果你想强制指定某个镜像，可以这样运行：

```bash
PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
PIP_TRUSTED_HOST=pypi.tuna.tsinghua.edu.cn \
./setup_env.sh
```

## 文件说明

- `setup_env.sh`：创建或复用 `.venv` 并安装依赖
- `test_waveshare_communication.py`：测试串口是否能与 Waveshare 控制板正常通信
- `test_motor_scan.py`：在多个波特率下扫描 Feetech 电机
- `test_single_motor.py`：控制单个电机，并提供交互式位置输入
- `test_open_close.py`：循环执行夹爪开合动作
- `lerobot_setup_motors.py`：调用 LeRobot 的设备配置流程来设置电机

## 环境要求

- Python 3.10 或更高版本
- 已连接的串口设备
- 可正常供电的 Feetech 电机 / Waveshare 控制板

## 手动安装

如果你不想使用 bash 脚本：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple \
  --trusted-host pypi.tuna.tsinghua.edu.cn \
  -r requirements.txt
```

如果你所在网络环境访问某个镜像慢或失败，可以把上面的镜像地址替换成阿里云镜像：

```bash
https://mirrors.aliyun.com/pypi/simple/
```

## 使用前需要修改的内容

这些脚本把串口和电机参数写死在文件顶部。运行前请先根据你的设备修改对应常量，例如：

- `PORT`
- `MOTOR_ID`
- `MOTOR_MODEL`
- `MOTOR_NAME`
- `BAUDRATE`

常见串口名称：

macOS：

```bash
/dev/cu.usbmodemXXXX
```

Linux：

```bash
/dev/ttyUSB0
```

## 如何使用

### 1. 测试 Waveshare 板卡通信

```bash
python test_waveshare_communication.py
```

这个脚本会打开指定串口，发送广播 ping，并输出是否收到返回数据。

### 2. 扫描电机 ID 和波特率

```bash
python test_motor_scan.py
```

这个脚本会遍历一组常见波特率，并尝试发现已连接的 Feetech 电机。

### 3. 控制单个电机

```bash
python test_single_motor.py
```

这个脚本会：

- 连接指定电机
- 尝试读取电压和当前位置
- 发送位置命令
- 进入交互模式，手动输入位置

如果 `MOTOR_NAME` 是 `gripper`，输入范围是 `0-100`。如果是其他普通关节，输入范围通常是 `-100` 到 `100`。

### 4. 连续执行夹爪开合

```bash
python test_open_close.py
```

这个脚本会反复执行打开和关闭动作，适合做简单稳定性测试。按 `Ctrl+C` 停止。

### 5. 使用 LeRobot 配置电机

示例：

```bash
python lerobot_setup_motors.py \
  --teleop.type=so100_leader \
  --teleop.port=/dev/tty.usbmodemXXXX
```

也可以根据你的设备改用 `robot` 参数。脚本内部支持的设备类型包括：

- `koch_follower`
- `koch_leader`
- `so100_follower`
- `so100_leader`
- `so101_follower`
- `so101_leader`
- `lekiwi`

## 常见问题

### 没有收到返回数据

请检查：

- 电机是否已单独供电
- 控制板 TX/RX 接线是否正确
- 串口名称是否正确
- 波特率是否正确
- 电机 ID 是否正确

### 出现 "There is no status packet"

这通常表示设备没有返回状态包，常见原因是：

- 串口不对
- 波特率不对
- 总线接线有问题
- 电机未上电

### 出现 overload / voltage 错误

这通常与以下问题有关：

- 电源电压不足
- 夹爪机械阻力过大
- 扭矩限制设置不合适
- 当前目标位置超出实际可运动范围

## 说明

这些脚本更接近调试工具，而不是可复用的 Python 包。如果后续会长期使用，比较实际的下一步是把硬编码的串口、电机 ID、波特率等参数改成命令行参数。
