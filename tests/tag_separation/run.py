#!/usr/bin/env python3
"""Execute extracted production callbacks with explicit non-ROS test doubles."""

import argparse
import hashlib
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


REFERENCE_SHA256 = 'c41f55f36d0c78dd37a59b5c6631fcf47103d9cb55154c7834cbab7f025938ff'


def extract_callback(source, name):
    start = source.index('void ' + name + '(')
    opening = source.index('{', start)
    level = 1
    for index in range(opening + 1, len(source)):
        level += (source[index] == '{') - (source[index] == '}')
        if level == 0:
            return source[start:index + 1]
    raise AssertionError('Unclosed callback: ' + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    fixture = (here / 'fixtures/reference_tag_localizer.cpp').read_bytes()
    assert hashlib.sha256(fixture).hexdigest() == REFERENCE_SHA256
    original = extract_callback(fixture.decode(), 'on_detections')
    current = extract_callback(
        (root / 'src/apriltag_localization/src/tag_localizer.cpp').read_text(),
        'on_observations')
    observer = extract_callback(
        (root / 'src/perception/src/tag_observer.cpp').read_text(), 'on_detections')
    # The entire localization equation, output construction, jump handling,
    # covariance assignment, and state updates remain byte-for-byte unchanged.
    equation = 'const tf2::Transform map_to_base ='
    assert original[original.index(equation):] == current[current.index(equation):]
    generated = '\n'.join([
        'struct ReferenceLocalizer : LocalizerState {\n' + original + '\n};',
        'struct CurrentLocalizer : LocalizerState {\n' + current + '\n};',
        'struct CurrentObserver : ObserverState {\n' + observer + '\n};',
    ])
    with tempfile.TemporaryDirectory(prefix='urrc-tag-regression-') as temporary:
        build = Path(temporary)
        (build / 'callbacks.hpp').write_text(generated)
        binary = build / 'tag_regression'
        command = shlex.split(os.environ.get('CXX', 'c++')) + [
            '-std=c++17', '-O1', '-g', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
        ]
        if args.sanitize:
            command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        command += ['-I' + str(build), str(here / 'test_pipeline.cpp'), '-o', str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
    print('PASS: fixed reference and localization calculation/output block are unchanged.')
    print('Scope: actual callback bodies with test doubles for messages, TF lookup, and transform math; '
          'not a ROS build, real tf2 test, DDS test, or live camera test.')


if __name__ == '__main__':
    main()
