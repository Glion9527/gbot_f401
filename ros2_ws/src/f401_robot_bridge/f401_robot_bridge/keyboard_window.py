"""Ubuntu Tk 窗口遥控；所有运动仍经过原 ROS 桥接和 STM32 保护。"""
# 与终端版共用ROS发布/服务实现，窗口版用按下和松开事件识别每次点按。
import time

import rclpy

from .hold_state import HoldState
from .keyboard_node import KeyboardNode


class WindowKeyboard(KeyboardNode):
    def __init__(self):
        # 复用父类的话题、服务客户端及50ms定时器；_tick在本类重写，不读取终端字符。
        super().__init__()
        self.input = HoldState(self._linear_speed, self._angular_speed)
        self.notice = '点击窗口后按 r 使能，再点按方向键'
        self._print_controls()

    def _print_controls(self):
        print('点按调档：W/X 前后、A/D 横移、Q/E 转向。S 停车，空格解除使能。', flush=True)
        print('松键保持目标速度。失焦或窗口卡顿会解除使能。', flush=True)
        print('失焦后若r无响应，先松开全部键，再点按并松开r一次，随后重新按r。', flush=True)
        print('失焦后方向键若无响应，先点按并松开该键，再按一次调档。', flush=True)

    def _service(self, arm):
        # 先让窗口输入归零，再复用父类异步服务；提示文字由服务Future更新。
        self.input.set_enabled(False)
        self.publish((0.0, 0.0, 0.0))
        previous = self._pending
        super()._service(arm)
        self.notice = '使能请求已发送，松开所有键并等待回复' if arm else '已请求停止并解除使能'
        if self._pending is previous:
            self.notice = ('上一条请求仍在处理中' if previous is not None and not previous.done()
                           else '服务未就绪，请先启动 network_bridge')
        elif self._pending is not None:
            generation = self._generation

            def show_result(future):
                # 与父类相同，过期请求不能覆盖更新的停止状态或使能提示。
                if generation != self._generation:
                    return
                try:
                    result = future.result()
                    self.notice = ('使能成功，松开所有键后再按方向键'
                                   if arm and result.success else str(result.message))
                except Exception as exc:
                    self.notice = '控制服务失败：' + str(exc)
            self._pending.add_done_callback(show_result)

    def key_press(self, key):
        # 使能回复可能刚到达，在首个周期发布前也应识别新的点按。
        self.input.set_enabled(self._enabled)
        action = self.input.press(key, time.monotonic())
        if action == 'arm':
            self._service(True)
        elif action == 'stop':
            self.force_stop('空格停止')
        elif action == 'zero':
            self.publish((0.0, 0.0, 0.0))

    def key_release(self, key):
        # 松键的短暂去抖由HoldState统一处理，UI层不自行猜测是否为自动重复。
        self.input.release(key, time.monotonic())

    def force_stop(self, reason):
        # 即使ARM回调尚未完成也必须调用_service(False)，让generation失效。
        self.input.halt()
        self._service(False)
        self.notice = reason

    def set_focus(self, focused):
        # 焦点影响运动许可：切换到其他窗口后立即请求停止，回焦后不会自动续跑。
        self.input.focus(focused)
        if not focused:
            self.force_stop('窗口失焦或最小化，松开全部键后重新按r')

    def before_spin(self):
        # 在ROS可能执行发布回调之前检查窗口处理间隔。
        reason = self.input.pump(time.monotonic())
        if reason:
            self.force_stop(reason)

    def _tick(self):
        # 50ms一次，发布已设定的档位速度或零速。
        self.input.set_enabled(self._enabled)
        self.publish(self.input.velocity(time.monotonic()))


def main(args=None):
    # 懒导入使纯状态测试和无GUI环境的模块检查不依赖Tk安装。
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as exc:
        raise SystemExit('缺少Tk，请在Ubuntu执行 sudo apt install python3-tk。') from exc
    rclpy.init(args=args)
    node = None
    window = None
    closing = False
    try:
        window = tk.Tk()
        window.title('F401 小车键盘控制')
        window.geometry('560x310')
        window.minsize(460, 280)
        node = WindowKeyboard()
        frame = ttk.Frame(window, padding=18)
        frame.pack(fill='both', expand=True)
        labels = []
        for text in ('r 使能 · S 停车 · 空格停车并解除使能',
                     'W/X 前后 ±1 档 · A/D 横移 ±1 档 · Q/E 转向 ±1 档\n每次点按调一档，松开保持目标速度。',
                     '失焦或最小化会停止。\n轮子需要换向时，按 S 等停稳后再调档。'):
            label = ttk.Label(frame, text=text, wraplength=500)
            label.pack(anchor='w', pady=(0, 10))
            labels.append(label)
        status = tk.StringVar(value=node.notice)
        label = ttk.Label(frame, textvariable=status, wraplength=500)
        label.pack(anchor='w')
        labels.append(label)
        frame.bind('<Configure>', lambda event: [item.configure(wraplength=max(120, event.width-36))
                                                for item in labels])

        def press(event):
            node.key_press(event.keysym)
            return 'break'

        def release(event):
            node.key_release(event.keysym)
            return 'break'

        def close():
            # closing使窗口关闭流程幂等，也阻止已经预约的pump继续工作。
            nonlocal closing
            if closing:
                return
            closing = True
            if rclpy.ok():
                node.force_stop('关闭窗口')
            else:
                node.input.halt()
                node._enabled = False
                node._generation += 1
            window.quit()

        def pump():
            # Tk拥有主事件循环，用非阻塞spin_once穿插处理ROS回调，避免启动第二个
            # 阻塞事件循环。after(10, ...)是重新预约，不会在此处睡眠10ms。
            if closing:
                return
            if not rclpy.ok():
                close()
                return
            # 必须在spin前检查卡顿；不可先续发一次旧的非零指令。
            node.before_spin()
            rclpy.spin_once(node, timeout_sec=0.0)
            permission = '输入已使能' if node._enabled else '输入未使能'
            forward, left, turn = node.input.velocity(time.monotonic())
            status.set(f'{permission} · {node.notice}\n'
                       f'目标  前后 {forward:+.2f} m/s  横移 {left:+.2f} m/s  '
                       f'转向 {turn:+.2f} rad/s')
            window.after(10, pump)

        window.bind('<KeyPress>', press)
        window.bind('<KeyRelease>', release)
        window.bind('<FocusIn>', lambda event: node.set_focus(True))
        window.bind('<FocusOut>', lambda event: node.set_focus(False))
        # 最小化未必产生FocusOut，单独处理顶层窗口取消映射。
        window.bind('<Unmap>', lambda event: node.set_focus(False)
                    if event.widget is window else None)
        window.bind('<Button-1>', lambda event: window.focus_set())
        window.protocol('WM_DELETE_WINDOW', close)
        window.after(50, window.focus_set)
        window.after(10, pump)
        window.mainloop()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        raise SystemExit('无法打开控制窗口，请在Ubuntu图形桌面终端运行：' + str(exc)) from exc
    finally:
        # 正常关窗、Ctrl-C、Tk异常均汇入清理路径，撤销输入后释放ROS和Tk资源。
        if node:
            node.input.halt()
            if rclpy.ok():
                node.stop_before_exit()
            node.destroy_node()
        if window:
            try:
                window.destroy()
            except tk.TclError:
                pass
        if rclpy.ok():
            rclpy.shutdown()
