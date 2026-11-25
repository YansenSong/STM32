"""Hold arrow keys in a focused window to control the vehicle over UDP."""

import socket
import tkinter as tk

from udp_send_cmd_test import STM32_IP, STM32_PORT, build_cmd


SPEED_MPS = 0.2
WHEEL_ANGLE_DEG = 3.0
PERIOD_MS = 20
NEUTRAL_PACKETS = 50
ARROWS = ("Up", "Down", "Left", "Right")


class KeyboardControl:
    def __init__(self, window, sock):
        self.window = window
        self.sock = sock
        self.keys = set()
        self.releases = {}
        self.counter = 0
        self.neutral_packets = 0
        self.closing = False
        self.status = tk.StringVar(value="等待方向键")

        window.title("车辆方向键控制")
        tk.Label(window, text="↑ 前进   ↓ 后退   ← 左转   → 右转", padx=24, pady=16).pack()
        tk.Label(window, text="松开方向键或切换窗口可停止控制 · Esc 退出", padx=24).pack()
        tk.Label(window, textvariable=self.status, pady=16).pack()
        for key in ARROWS:
            window.bind(f"<KeyPress-{key}>", self.press)
            window.bind(f"<KeyRelease-{key}>", self.release)
        window.bind("<FocusOut>", self.stop)
        window.bind("<Escape>", self.close)
        window.protocol("WM_DELETE_WINDOW", self.close)
        window.after(PERIOD_MS, self.tick)

    def press(self, event):
        key = event.keysym
        pending = self.releases.pop(key, None)
        if pending is not None:
            self.window.after_cancel(pending)
        if not self.closing and self.window.focus_get() is not None:
            self.keys.add(key)
            self.neutral_packets = NEUTRAL_PACKETS

    def release(self, event):
        key = event.keysym
        # X11 auto-repeat can enqueue a release followed by a press.
        if key in self.keys and key not in self.releases:
            self.releases[key] = self.window.after_idle(self.finish_release, key)

    def finish_release(self, key):
        self.releases.pop(key)
        self.keys.discard(key)

    def stop(self, _event=None):
        for pending in self.releases.values():
            self.window.after_cancel(pending)
        self.releases.clear()
        self.keys.clear()

    def close(self, _event=None):
        self.closing = True
        self.stop()

    def tick(self):
        if self.window.focus_get() is None:
            self.stop()
        if self.keys or self.neutral_packets:
            speed = SPEED_MPS * (("Up" in self.keys) - ("Down" in self.keys))
            angle = WHEEL_ANGLE_DEG * (("Right" in self.keys) - ("Left" in self.keys))
            data = build_cmd(group=0x03, param1=round(speed * 10000),
                             param2=round(angle * 10000), subcmd=self.counter)
            if not self.keys:
                self.neutral_packets -= 1
            try:
                self.sock.sendto(data, (STM32_IP, STM32_PORT))
            except OSError as error:
                self.stop()
                self.status.set(f"UDP 发送失败：{error}")
            else:
                self.counter = (self.counter + 1) & 0xFF
                self.status.set(f"速度目标 {speed:+.2f} m/s   车轮目标 {angle:+g}°")
        elif self.closing:
            self.window.destroy()
            return
        else:
            self.status.set("等待方向键")
        self.window.after(PERIOD_MS, self.tick)


def main():
    try:
        window = tk.Tk()
    except tk.TclError as error:
        print(f"无法打开控制窗口：{error}")
        return 1
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setblocking(False)
        KeyboardControl(window, sock)
        window.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
