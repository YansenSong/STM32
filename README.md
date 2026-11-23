# STM32 车辆域控（CAN / 遥控 / UDP）

当前工程使用 STM32F407ZG 和 FreeRTOS。Vehicle 应用任务负责通信轮询，并以 20 ms 周期处理车辆控制及状态回传。

## 目录

```text
Application/
  Core/inc/       HAL 配置、中断声明
  Core/src/       中断处理
  User/inc/       车辆配置、协议和模块接口
  User/src/       当前运行代码
Libraries/        ST HAL 与 CMSIS 库
docs/             原理图、UDP 原始说明及 CAN 故障复盘
tools/            UDP 指令发送、状态接收脚本
```

## 构建状态

工程内置 FreeRTOS-Kernel V11.2.0（GCC ARM_CM4F 移植层）。Vehicle 任务每轮结束后等待一个 1 ms tick，Health 任务以较低优先级每秒生成一次诊断报告，FreeRTOS 另有一个空闲任务。任务栈静态分配，TIM2 保留为独立的微秒计时基准。

SysTick 同时推进 HAL 和 FreeRTOS 的毫秒计时。串口 ISR 继续只写接收缓冲区，不调用 FreeRTOS API。可通过 J-Link 观察 `dbg_freertos_assert_line`、`dbg_freertos_stack_overflow` 和 `dbg_vehicle_stack_free_words`。
Keil 工程及下载调试配置已移除。现已补充 GNU Arm 工具链的 Makefile、启动文件、链接脚本与 J-Link SWD 下载配置。
库内的 ARM 编译器适配头和启动模板作为第三方库内容保留，供工具链迁移时参考。

### GNU Arm 编译与 J-Link 烧录

需要 `arm-none-eabi-gcc`、配套 newlib（含 nano.specs）、GNU Make 和 SEGGER J-Link Commander，并确保终端能够访问 USB J-Link。
Ubuntu 可安装 `gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi`。

```sh
make -j
make flash
```

输出在 `build/vehicle.elf`、`build/vehicle.hex`、`build/vehicle.bin`，映射文件为 `build/vehicle.map`。
可用 `make CROSS_COMPILE=/工具链路径/bin/arm-none-eabi-` 指定工具链。
下载目标为 STM32F407ZG，SWD 速度为 1000 kHz；写入后校验、复位并运行。
链接使用 1 MiB Flash 和 128 KiB 普通 SRAM，保留至少 4 KiB 栈空间；以太网 DMA 缓冲不放入 CCM。

## OTA 启动原型

`make ota` 生成 `build/ota/` 下的 boot、slot-a、slot-b、meta-a 和 meta-b 的 HEX 文件。
`make flash` 仍写入从 Flash 基地址启动的单镜像固件。已有 Bootloader 的板子上执行该命令会覆盖 Bootloader。

| 区域 | 起始地址 | 大小 |
| --- | --- | --- |
| Bootloader（扇区 0–3） | `0x08000000` | 64 KiB |
| A 元数据（扇区 4） | `0x08010000` | 64 KiB |
| A 固件（扇区 5–7） | `0x08020000` | 384 KiB |
| B 固件（扇区 8–10） | `0x08080000` | 384 KiB |
| B 元数据（扇区 11） | `0x080E0000` | 128 KiB |

示例元数据将 A 标记为已确认版本 1，B 标记为候选版本 2。
Bootloader 检查向量、镜像范围和 CRC32 后启动 B 一次。B 在运行具备以太网链路且无控制漏周期时，
于上电约 5 秒后确认。确认之前如果复位，Bootloader 会回到 A。
看门狗超时取决于 LSI 频率，标称约 32 秒。可用 J-Link 读取应用的
`dbg_ota_image_version` 和 `dbg_ota_confirmed`。

当前阶段的镜像和元数据由可信电脑本地生成。CRC32 用于检测损坏。
`make ota-signed OTA_PUBLIC_KEY=/path/to/public.pem OTA_SIGN_KEY=/path/to/private.pem`
在 `build/ota/signed/` 生成强制 Ed25519 验签的 Bootloader 和元数据。
使用 OpenSSL 的 `genpkey -algorithm ED25519` 创建私钥，再用 `pkey -pubout` 导出公钥。
私钥由签发端保存，Bootloader 仅嵌入公钥。签名绑定槽地址、长度、CRC32、版本和镜像 SHA-512。
签名 Bootloader 要与签名元数据一起部署，普通 `make ota` 产物用于先前的 CRC 台架原型。
局域网上传使用 UDP 5003。当前为台架原型，断开执行器后，通过 J-Link 对
`dbg_ota_arm_request` 写入 `0x4F544121` 才开启维护窗口。维护期间暂停 CAN 控制输出及控制输入处理，
只保留必要的以太网处理和升级应答。Update 任务负责擦写，Vehicle 任务保留网口所有权。
30 秒无升级报文会复位退出。已确认槽在升级过程中保留，候选槽元数据在传输完整且 CRC 正确后发布，
签名由 Bootloader 在启动前验证。

`python3 tools/ota_upload.py IMAGE.bin META.hex --reboot` 发起上传。目标必须是当前未运行的槽，
签发镜像版本应大于当前已确认版本。可用 `ota_manifest.py` 的版本参数和 `--sign-key`
生成候选元数据。正式启用之前要用 J-Link 一次性部署含 Update 任务的应用和签名 Bootloader。

## 代码入口与配置

- `Application/User/src/main.c`：初始化、通信轮询及 20ms 控制周期。
- `Application/User/inc/vehicle_config.h`：引脚、CAN、UDP、遥控及协议参数。
- `bsp_can.c` / `vehicle_feedback.c`：CAN 收发与反馈解析。
- `bsp_uart.c` / `upper_cmd.c`：串口接收和 26 字节指令解析。
- `bsp_eth.c`：轻量 ARP / IPv4 / UDP 通信。
- `remote_control.c`：遥控接管。
- `vehicle_control.c` / `vehicle_status.c`：车辆控制与状态回传。

当前入口使用 HSI 16MHz；CAN 时序以 `bsp_can.c` 实现为准，配置头中的旧 42MHz 注释尚未同步。

## 调试脚本

`python3 tools/udp_health_test.py --duration 10` 接收 UDP 5002 的诊断报告。
Vehicle 和 Health 通过两个长度为 1 的静态队列交换最新诊断快照和报告。
Vehicle 统一发送 UDP，链路恢复后发送最新报告。UDP 5001 的状态协议保持原格式。
报告中的 `ethernet_retries` 统计自启动以来触发的 Ethernet 重试，`can_online_mask` 记录仍有新鲜反馈的 CAN 来源。

在项目根目录运行 `python3 tools/udp_send_cmd_test.py` 或 `python3 tools/udp_status_test.py`。
运行前检查脚本内的 IP 和端口；发送脚本含车辆动作指令，需在具备调试条件时手动执行。

## 本次整理

旧 FreeRTOS、通信驱动、电机控制、回溯模块、备用入口、重复补丁、编译产物及个人 IDE 布局已归档。
归档位于工程上一级的 `项目归档/`，含逐文件 SHA-256 校验过的压缩包及迁移清单。
恢复时可从压缩包按原相对路径取回文件。文档和脚本的新位置见同目录清单。
保留完整 HAL/CMSIS 库，业务代码与通信协议未修改，Git 历史保留，未执行提交。

Keil 工程及调试配置另存于上一级 `项目归档/Keil配置-*.tar.gz`，可按需恢复。
