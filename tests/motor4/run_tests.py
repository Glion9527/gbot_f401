#!/usr/bin/env python3
"""Compile the project's Core motor sources against a small host HAL model.

Only a temporary directory receives executable/C build files. The STM32 project
sources are never changed by this runner.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import re

MODULE_FILES = ('motor4.c', 'motor4.h', 'mecanum.c', 'mecanum.h',
                'robot_app.c', 'robot_app.h')


def main():
    tests = Path(__file__).resolve().parent
    project = Path(__file__).resolve().parents[2]
    needed = [project / 'Core' / ('Src' if name.endswith('.c') else 'Inc') / name
              for name in MODULE_FILES]
    print(f'Testing Core sources: {project / "Core"}', flush=True)
    needed.append(tests / 'test_motor4.c.txt')
    for source in needed:
        if not source.is_file():
            raise SystemExit(f'Missing source: {source}')
    compiler = shutil.which('clang')
    if compiler is None:
        raise SystemExit('clang was not found; install the Xcode Command Line Tools.')
    with tempfile.TemporaryDirectory(prefix='stm32-motor4-tests-') as directory:
        build = Path(directory)
        for source in [*needed, tests / 'main.h.txt', tests / 'tim.h.txt', tests / 'usart.h.txt']:
            shutil.copyfile(source, build / source.name.removesuffix('.txt'))
        # Only the temporary copy receives diagnostic instrumentation: emulate
        # MCU reset/fault input and count calls into the real SetAll body.
        module = build / 'motor4.c'
        instrumented, count = re.subn(
            r'(bool Motor4_SetAllSpeedsRPM\([^)]*\)\s*\{)',
            r'\1\n    extern unsigned host_set_all_calls;\n    ++host_set_all_calls;',
            module.read_text())
        if count != 1:
            raise SystemExit('Could not instrument the real SetAll command entry.')
        instrumented += '''
void Host_EmulateMCUReset(void) { s_initialized = false; }
void Host_InjectFault(unsigned source, uint8_t fault) {
    uint32_t key = Lock();
    LatchFault(source, fault);
    Unlock(key);
}
'''
        module.write_text(instrumented)
        app_header = (build / 'robot_app.h').read_text()
        scenarios = [(f'app-mode-{mode}', mode, []) for mode in range(7)]
        scenarios.append(('sign-overrides', 0,
                          ['-DHOST_SIGN_OVERRIDES=1', '-DMOTOR4_FL_ENCODER_SIGN=1',
                           '-DMOTOR4_FR_DRIVE_SIGN=1']))
        for name, mode, defines in scenarios:
            mode_header, count = re.subn(r'(?m)^#define ROBOT_APP_MODE\s+\d+\s*$',
                                        f'#define ROBOT_APP_MODE {mode}', app_header)
            if count != 1:
                raise SystemExit('Could not select RobotApp mode in the temporary header.')
            (build / 'robot_app.h').write_text(mode_header)
            executable = build / name
            command = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                       '-Wconversion', '-Wno-sign-conversion', '-fsanitize=undefined,address',
                       '-fno-omit-frame-pointer', '-g', '-I', str(build), *defines,
                       str(build / 'motor4.c'), str(build / 'mecanum.c'),
                       str(build / 'robot_app.c'), str(build / 'test_motor4.c'), '-lm',
                       '-o', str(executable)]
            subprocess.run(command, check=True)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            if result.returncode:
                print(result.stdout, end='')
                print(result.stderr, end='')
                raise SystemExit(f'{name} failed with return code {result.returncode}')
            if name == 'app-mode-0':
                print(result.stdout, end='')
            else:
                print(f'{name}: {result.stdout.splitlines()[-1]}')
    print('Host verification only: no STM32 compilation, flashing, or hardware test was performed.')


if __name__ == '__main__':
    main()
