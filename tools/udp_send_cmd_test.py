import socket
import struct
import time

STM32_IP = "192.168.5.50"
STM32_PORT = 5000

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
# EPS 右转 30度：当前 STM32 串口协议仍按角度*10000解析，所以 Param2=300000
cmd_eps_right_30 = build_cmd(group=0x02, param2=300000, subcmd=0x0C)
# EPS 回中
cmd_eps_center = build_cmd(group=0x02, param2=0, subcmd=0x0A)
# SEB 压力刹车 0.2MPa：Param3 = 0.2*100 = 20
cmd_seb_pressure = build_cmd(group=0x04, flag=0x01, param3=20, subcmd=0x50)
# SEB 释放
cmd_seb_release = build_cmd(group=0x04, flag=0x01, param3=0, subcmd=0x65)

cmds = {
    "1": ("RT49 前进 0.2m/s", cmd_rt49_forward),
    "2": ("RT49 停止", cmd_rt49_stop),
    "3": ("EPS 右转 30度", cmd_eps_right_30),
    "4": ("EPS 回中", cmd_eps_center),
    "5": ("SEB 压力刹车 0.2MPa", cmd_seb_pressure),
    "6": ("SEB 释放", cmd_seb_release),
}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    print("\n选择要通过 UDP 发送的 26 字节命令：")
    for k, (name, _) in cmds.items():
        print(f"{k} - {name}")
    print("q - 退出")

    key = input("请输入：").strip()
    if key.lower() == "q":
        break
    if key not in cmds:
        print("无效输入")
        continue

    name, data = cmds[key]
    print("发送：", name)
    print("HEX:", hexstr(data))
    sock.sendto(data, (STM32_IP, STM32_PORT))
    time.sleep(0.05)

sock.close()
