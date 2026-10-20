#!/usr/bin/env python3
"""Exercise already-granted Docker group access without changing real groups."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1]

DOCKER = r'''#!/usr/bin/env python3
import json, os, pathlib, sys
root = pathlib.Path(os.environ['MOCK_STATE'])
args = sys.argv[1:]
with (root / 'docker.calls').open('a') as output:
    output.write(json.dumps({'args': args, 'group_access': os.getenv('MOCK_GROUP_ACCESS')}) + '\n')
if os.getenv('MOCK_DENY_ALL') == '1' or os.getenv('MOCK_GROUP_ACCESS') != '1':
    sys.exit(1)
if args[0] == 'inspect':
    sys.exit(1)
if args[:2] == ['image', 'inspect']:
    if args[-1] == 'px4-plotjuggler-live:humble-2.4.2' and not (root / 'built').exists():
        sys.exit(1)
    if '--format' in args:
        print('sha256:mock-base-image')
elif args[0] == 'build':
    (root / 'built').touch()
'''

SG = r'''#!/usr/bin/env python3
import json, os, pathlib, sys
root = pathlib.Path(os.environ['MOCK_STATE'])
with (root / 'sg.calls').open('a') as output:
    output.write(json.dumps(sys.argv[1:]) + '\n')
assert len(sys.argv) == 4 and sys.argv[2] == '-c'
environment = os.environ.copy()
if sys.argv[1] == 'docker':
    environment['MOCK_GROUP_ACCESS'] = '1'
os.execve('/bin/sh', ['sh', '-c', sys.argv[3]], environment)
'''


class DockerGrantedGroupsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='px4-monitor-groups-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.repo = self.root / 'px4'
        (self.repo / 'msg').mkdir(parents=True)
        self.state = self.root / 'state'
        self.env = os.environ.copy()
        self.env.update(PATH=str(self.bin) + ':' + os.environ['PATH'],
                        MOCK_STATE=str(self.root), MOCK_DOCKER_MEMBER='1',
                        PJ_PX4_DIR=str(self.repo), PJ_STATE_DIR=str(self.state),
                        PJ_CONFIG=str(self.root / 'no-config'))
        self.env.pop('MOCK_GROUP_ACCESS', None)
        self.executable('docker', DOCKER)
        self.executable('sg', SG)
        self.executable('getent', '#!/bin/bash\nif [[ "$*" == "group docker" ]]; then\n'
                        ' echo "docker:x:134:mock-user"\nelse exec /usr/bin/getent "$@"; fi\n')
        self.executable('id', '#!/bin/bash\nif [[ "$1" == -G && $# == 2 ]]; then\n'
                        ' if [[ "$MOCK_DOCKER_MEMBER" == 1 ]]; then echo "1000 134"; else echo 1000; fi\n'
                        'else exec /usr/bin/id "$@"; fi\n')
        self.executable('sudo', '#!/bin/bash\nexit 1\n')

    def executable(self, name, content):
        path = self.bin / name
        path.write_text(content)
        path.chmod(0o755)

    def run_common(self, command, *arguments):
        return subprocess.run(['bash', '-c', 'source "$1"; ' + command,
                               'test', str(TOOLS / 'common.sh'), *arguments],
                              env=self.env, capture_output=True, text=True,
                              stdin=subprocess.DEVNULL, timeout=15)

    def calls(self, name):
        path = self.root / (name + '.calls')
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def test_granted_group_is_used_only_for_docker_and_arguments_are_preserved(self):
        sentinel = self.root / 'must-not-be-created'
        argument = "space ' quote \" $(touch " + str(sentinel) + ") `false`\nsecond line"
        result = self.run_common(
            'before=$(id); select_docker; docker_cmd custom "$2"; '
            '[[ "$(id)" == "$before" ]]', argument)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.calls('docker')[-1]['args'], ['custom', argument])
        self.assertEqual(self.calls('docker')[-1]['group_access'], '1')
        groups = self.calls('sg')
        self.assertEqual(len(groups), 4)  # one bounded preflight, one client call
        self.assertEqual(groups[0][0], 'docker')
        self.assertEqual(groups[2][0], 'docker')
        primary = subprocess.check_output(['/usr/bin/id', '-gn'], text=True).strip()
        self.assertEqual(groups[1][0], primary)
        self.assertEqual(groups[3][0], primary)

    def test_ungranted_group_is_never_attempted(self):
        self.env['MOCK_DOCKER_MEMBER'] = '0'
        result = self.run_common('select_docker')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls('sg'), [])
        self.assertNotIn('Password:', result.stderr)

    def test_unavailable_daemon_fails_without_repeated_group_attempts(self):
        self.env['MOCK_DENY_ALL'] = '1'
        result = self.run_common('select_docker')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(self.calls('sg')), 2)
        self.assertIn('Docker daemon is unavailable', result.stderr)

    def test_existing_access_does_not_switch_groups(self):
        self.env['MOCK_GROUP_ACCESS'] = '1'
        result = self.run_common('select_docker; docker_cmd info')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.calls('sg'), [])

    def test_setup_first_image_build_uses_the_same_granted_group(self):
        result = subprocess.run(['bash', str(TOOLS / 'setup.sh')], env=self.env,
                                capture_output=True, text=True, stdin=subprocess.DEVNULL,
                                timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((self.root / 'built').exists())
        self.assertTrue((self.state / 'image-build.tsv').exists())
        calls = self.calls('docker')
        self.assertTrue(any(call['args'][0] == 'build' for call in calls))
        self.assertTrue(any(call['args'][-1] == 'prepare' for call in calls))
        for call in calls:
            if call['args'][0] != 'info':
                self.assertEqual(call['group_access'], '1', call)


if __name__ == '__main__':
    unittest.main()
