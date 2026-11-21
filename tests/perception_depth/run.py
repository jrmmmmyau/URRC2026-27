#!/usr/bin/env python3
"""Run camera-coordinate depth helpers without ROS."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    with tempfile.TemporaryDirectory(prefix='urrc-perception-depth-') as temporary:
        build = Path(temporary)
        binary = build / 'perception_depth'
        command = shlex.split(os.environ.get('CXX', 'c++')) + [
            '-std=c++17', '-O1', '-g', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
        ]
        if args.sanitize:
            command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        command += ['-I' + str(build), '-I' + str(root / 'src/perception/include'),
                    str(here / 'test_pipeline.cpp'),
                    '-o', str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
    print('Scope: production camera depth sampling and encoding helpers; '
          'plain message containers replace ROS messages. No live ROS verification.')


if __name__ == '__main__':
    main()
