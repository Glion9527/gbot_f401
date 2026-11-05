#!/usr/bin/env python3
"""对真实遥控/麦轮/电机源码进行主机 HAL 仿真，不连接开发板。"""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--emit-fixture', type=Path,
                        help='also save actual firmware ACK/T lines for cross-language tests')
    args = parser.parse_args()
    tests = Path(__file__).resolve().parent
    project = tests.parents[1]
    compiler = shutil.which('clang')
    if compiler is None:
        raise SystemExit('clang was not found; install Xcode Command Line Tools.')
    sources = ['motor4.c', 'motor4.h', 'mecanum.c', 'mecanum.h',
               'robot_app.h', 'remote_control.c', 'remote_control.h']
    with tempfile.TemporaryDirectory(prefix='stm32-remote-tests-') as folder:
        build = Path(folder)
        for name in sources:
            src = project/'Core'/('Src' if name.endswith('.c') else 'Inc')/name
            shutil.copyfile(src, build/name)
        for name in ['main.h', 'tim.h']:
            shutil.copyfile(project/'tests/motor4'/(name+'.txt'), build/name)
        for name in ['usart.h', 'test_remote_control.c']:
            shutil.copyfile(tests/(name+'.txt'), build/name)
        motor = build/'motor4.c'
        motor.write_text(motor.read_text()+'''
void Host_EmulateMCUReset(void) { s_initialized = false; }
void Host_InjectFault(unsigned source, uint8_t fault) {
    uint32_t key = Lock(); LatchFault(source, fault); Unlock(key);
}
void Host_SetPosition(unsigned wheel, int64_t position) {
    uint32_t key = Lock(); s_motor[wheel].position = position; Unlock(key);
}
''')
        binary = build/'remote-tests'
        command = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                   '-Wconversion', '-Wno-sign-conversion',
                   '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                   '-g', '-I', str(build), str(build/'motor4.c'),
                   str(build/'mecanum.c'), str(build/'remote_control.c'),
                   str(build/'test_remote_control.c'), '-lm', '-o', str(binary)]
        subprocess.run(command, check=True)
        env = dict(os.environ)
        if args.emit_fixture is not None:
            env['REMOTE_CONTROL_FIXTURE'] = str(args.emit_fixture.resolve())
        subprocess.run([str(binary)], check=True, env=env)
    print('Host verification only: no physical serial port, flashing or movement.')


if __name__ == '__main__':
    main()
