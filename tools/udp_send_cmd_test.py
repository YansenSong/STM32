import math
import socket
import struct
import time

STM32_IP = "192.168.5.50"
STM32_PORT = 5000
MAX_WHEEL_ANGLE_DEG = 46.2  # Firmware limit: 462 EPS degrees at 10:1.
STEERING_TEST_SECONDS = 5.0
STEERING_PERIOD_SECONDS = 0.02

def build_cmd(group, param1=0, param2=0, flag=0, param3=0, subcmd=0):
    frame = bytearray()
    frame += b"cmd__"
    frame += bytes([0x01, 0x01, 0x00, group & 0xFF])
    frame += struct.pack(">i", int(param1))
    frame += struct.pack(">i", int(param2))
    frame += bytes([flag & 0xFF])
    frame += struct.pack(">h", int(param3))
    frame += bytes([0x00, subcmd & 0xFF])
    checksum = sum(frame) & 0xFFFFFFFF
    frame += struct.pack(">I", checksum)
    assert len(frame) == 26
    return bytes(frame)

def hexstr(data):
    return " ".join(f"{x:02X}" for x in data)

# RT49 前进 0.2m/s：Param1 = 0.2*10000 = 2000
cmd_rt49_forward = build_cmd(group=0x01, param1=2000, subcmd=0x04)
# RT49 停止
cmd_rt49_stop = build_cmd(group=0x01, param1=0, subcmd=0x01)
# 车轮右转 3 度：Param2 为车轮角度 * 10000，固件换算为 EPS 目标 30 度。
cmd_eps_right_3 = build_cmd(group=0x02, param2=30000, subcmd=0x0C)
# EPS 回中
cmd_eps_center = build_cmd(group=0x02, param2=0, subcmd=0x0A)
# SEB 压力刹车 0.2MPa：Param3 = 0.2*100 = 20
cmd_seb_pressure = build_cmd(group=0x04, flag=0x01, param3=20, subcmd=0x50)
# SEB 释放
cmd_seb_release = build_cmd(group=0x04, flag=0x01, param3=0, subcmd=0x65)

cmds = {
    "1": ("RT49 前进 0.2m/s", cmd_rt49_forward),
    "2": ("RT49 停止", cmd_rt49_stop),
    "3": ("车轮右转 3 度（新版固件）", cmd_eps_right_3),
    "4": ("EPS 回中", cmd_eps_center),
    "5": ("SEB 压力刹车 0.2MPa", cmd_seb_pressure),
    "6": ("SEB 释放", cmd_seb_release),
}

def send_steering(sock, wheel_angle_deg, duration, first_counter):
    deadline = time.monotonic() + duration
    counter = first_counter
    sent = 0
    while (now := time.monotonic()) < deadline:
        data = build_cmd(group=0x02, param2=round(wheel_angle_deg * 10000),
                         subcmd=counter)
        sock.sendto(data, (STM32_IP, STM32_PORT))
        counter = (counter + 1) & 0xFF
        sent += 1
        time.sleep(max(0.0, min(deadline, now + STEERING_PERIOD_SECONDS) - time.monotonic()))
    return sent


def test_steering(sock):
    try:
        angle = float(input("车轮目标角度（度，正值向右，负值向左）：").strip())
    except ValueError:
        print("请输入有效数字。")
        return
    if not math.isfinite(angle) or abs(angle) > MAX_WHEEL_ANGLE_DEG:
        print(f"车轮角度须在 ±{MAX_WHEEL_ANGLE_DEG:g}° 内。")
        return

    print(f"发送车轮目标 {angle:g}°，持续 {STEERING_TEST_SECONDS:g} 秒。按 Ctrl+C 提前回正。")
    try:
        sent = send_steering(sock, angle, STEERING_TEST_SECONDS, first_counter=0)
        print(f"已发送 {sent} 条目标指令。")
    finally:
        print("发送 0° 回正指令。")
        sent = send_steering(sock, 0, 1.0, first_counter=0)
        print(f"已发送 {sent} 条回正指令。请观察车轮或状态反馈确认回正。")


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            while True:
                print("\n选择要通过 UDP 发送的 26 字节命令：")
                for key, (name, _) in cmds.items():
                    print(f"{key} - {name}")
                print("7 - 输入车轮角度，持续测试后回正（新版固件）")
                print("q - 退出")

                key = input("请输入：").strip().lower()
                if key == "q":
                    break
                if key == "7":
                    test_steering(sock)
                    continue
                if key not in cmds:
                    print("无效输入")
                    continue

                name, data = cmds[key]
                print("发送：", name)
                print("HEX:", hexstr(data))
                sock.sendto(data, (STM32_IP, STM32_PORT))
                time.sleep(0.05)
        except (EOFError, KeyboardInterrupt):
            print("\n退出。")
        except OSError as error:
            print(f"UDP 发送失败：{error}")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
