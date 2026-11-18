#!/usr/bin/env python3
"""对真实 link_v2/motor4 源码做主机 HAL 仿真，不连接开发板。"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def main():
    tests = Path(__file__).resolve().parent
    project = tests.parents[1]
    compiler = shutil.which('clang')
    if compiler is None:
        raise SystemExit('clang was not found; install Xcode Command Line Tools.')

    sources = ['motor4.c', 'motor4.h', 'link_v2.c', 'link_v2.h', 'robot_app.h']
    with tempfile.TemporaryDirectory(prefix='stm32-link-v2-tests-') as folder:
        build = Path(folder)
        for name in sources:
            src = project / 'Core' / ('Src' if name.endswith('.c') else 'Inc') / name
            shutil.copyfile(src, build / name)

        # 契约头：测试目录里直接放协议定义，避免复制转发头的相对路径问题。
        shutil.copyfile(project / 'protocol' / 'link_v2.h',
                        build / 'link_v2_contract.h')
        shutil.copyfile(project / 'tests/motor4/main.h.txt', build / 'main.h')
        shutil.copyfile(project / 'tests/motor4/tim.h.txt', build / 'tim.h')
        shutil.copyfile(tests / 'usart.h.txt', build / 'usart.h')
        shutil.copyfile(tests / 'test_link_v2.c.txt', build / 'test_link_v2.c')

        binary = build / 'link-v2-tests'
        command = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                   '-Wconversion', '-Wno-sign-conversion',
                   '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                   '-DLINK_V2_HOST_TEST', '-g', '-I', str(build),
                   str(build / 'motor4.c'), str(build / 'link_v2.c'),
                   str(build / 'test_link_v2.c'), '-lm', '-o', str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
    print('Host verification only: no physical serial port, flashing or movement.')


if __name__ == '__main__':
    main()
