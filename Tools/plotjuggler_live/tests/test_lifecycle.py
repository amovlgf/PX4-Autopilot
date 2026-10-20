#!/usr/bin/env python3
"""Exercise host lifecycle scripts with a fake Docker daemon and a real inert PID.

No ROS, GUI, flight process, Docker daemon, or elevated privileges are needed.
"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import tempfile
import unittest


TOOLS = Path(__file__).resolve().parents[1]
FAKE_DOCKER = r'''#!/usr/bin/env python3
import json, os, pathlib, sys
root = pathlib.Path(os.environ['MOCK_STATE'])
state_file = root / 'docker.json'
state = json.loads(state_file.read_text()) if state_file.exists() else {}
args = sys.argv[1:]
with (root / 'docker.calls').open('a') as output:
    output.write(json.dumps(args) + '\n')
command = args[0]
if command in ('info', 'build') or args[:2] == ['image', 'inspect']:
    sys.exit(0)
if command == 'inspect':
    if not state:
        sys.exit(1)
    format = args[2]
    if '.launch' in format:
        print(state.get('launch', ''))
    elif '.Config.Labels' in format:
        print(state['owner'])
    elif '.State.Running' in format:
        print(str(state.get('running', False)).lower())
    else:
        print(state['id'])
elif command == 'run':
    if '--detach' in args:
        labels = [args[i + 1] for i, item in enumerate(args) if item == '--label']
        owner = next(item.split('=', 1)[1] for item in labels if '.owner=' in item)
        launch = next(item.split('=', 1)[1] for item in labels if '.launch=' in item)
        state = {'id': 'a' * 64, 'owner': owner, 'launch': launch, 'running': os.environ.get('MOCK_START_FAIL') != '1'}
        if os.environ.get('MOCK_FOREIGN_LAUNCH') == '1':
            state['launch'] = 'another-attempt'
        state_file.write_text(json.dumps(state))
        if os.environ.get('MOCK_RUN_FAIL_NO_ID') == '1':
            sys.exit(125)
        print(state['id'])
elif command == 'exec':
    failed = (args[-1] == 'probe' and os.environ.get('MOCK_PROBE_FAIL') == '1') or (args[-1] == 'diagnose' and os.environ.get('MOCK_DIAGNOSE_FAIL') == '1')
    sys.exit(1 if failed else 0)
elif command == 'stop':
    state['running'] = False
    state_file.write_text(json.dumps(state))
elif command == 'rm':
    state_file.unlink(missing_ok=True)
elif command == 'logs':
    print('test container log')
else:
    sys.exit('Unhandled fake docker command: ' + repr(args))
'''


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.extra_processes = []
        self.temporary = tempfile.TemporaryDirectory(prefix='px4-monitor-test-')
        self.root = Path(self.temporary.name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.repo = self.root / 'px4'
        (self.repo / 'msg').mkdir(parents=True)
        px4_bin = self.repo / 'build/px4_sitl_default/bin'
        px4_bin.mkdir(parents=True)
        # Executable identity is real; the process just sleeps, never flies.
        shutil.copy2('/bin/sleep', px4_bin / 'px4')
        self.sim = subprocess.Popen([str(px4_bin / 'px4'), '90'])
        self.write_executable(self.bin / 'docker', FAKE_DOCKER)
        self.write_executable(self.bin / 'ss', '#!/bin/bash\nprintf "%s" "${MOCK_PORT:-}"\n')
        self.write_executable(px4_bin / 'px4-uxrce_dds_client',
                              '#!/bin/bash\necho "Running, disconnected"\necho "Agent IP: 127.0.0.1"\necho "Agent port: 8888"\n')
        self.write_executable(px4_bin / 'px4-param',
                              '#!/bin/bash\necho "x UXRCE_DDS_DOM_ID [1,2] : ${MOCK_DOMAIN:-0}"\n')
        self.state = self.root / 'state'
        (self.state / 'current/install').mkdir(parents=True)
        (self.state / 'current/install/setup.bash').touch()
        self.env = os.environ.copy()
        self.env.update(PATH=str(self.bin) + ':' + os.environ['PATH'], PJ_PX4_DIR=str(self.repo),
                        PJ_STATE_DIR=str(self.state), PJ_CONFIG=str(self.root / 'absent.env'),
                        MOCK_STATE=str(self.root), ROS_DOMAIN_ID='0', PJ_DDS_PORT='8888')

    def tearDown(self):
        for process in self.extra_processes:
            process.terminate()
            process.wait(timeout=5)
        self.sim.terminate()
        self.sim.wait(timeout=5)
        self.temporary.cleanup()

    @staticmethod
    def write_executable(path, content):
        path.write_text(content)
        path.chmod(0o755)

    def run_script(self, name, *args, success=True):
        result = subprocess.run(['bash', str(TOOLS / name), *args], env=self.env,
                                text=True, capture_output=True, timeout=15)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def calls(self):
        return [json.loads(line) for line in (self.root / 'docker.calls').read_text().splitlines()]

    def test_attach_repeat_and_stop_preserve_simulation(self):
        self.run_script('start.sh', '--attach', '--no-gui')
        self.run_script('start.sh', '--attach', '--no-gui')
        self.assertEqual(sum(call[0] == 'run' for call in self.calls()), 1)
        runtime = json.loads((self.state / 'runtime.json').read_text())
        self.assertEqual(runtime['px4_pid'], str(self.sim.pid))
        self.assertTrue(runtime['px4_start_ticks'])
        self.assertFalse(runtime['px4_owned'])
        self.run_script('stop.sh')
        self.run_script('stop.sh')
        self.assertIsNone(self.sim.poll())
        self.assertEqual(sum(call[0] == 'stop' for call in self.calls()), 1)
        self.assertTrue((self.state / 'logs/container.log').exists())

    def external_agent_fixture(self, version):
        library = self.root / ('libmicroxrcedds_agent.so.' + version)
        library.write_bytes(b'\0' * 4096)
        ready = self.root / 'mapped-library-ready'
        process = subprocess.Popen([sys.executable, '-c',
            'import mmap,pathlib,sys,time; f=open(sys.argv[1],"rb"); '
            'mapped=mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ); '
            'pathlib.Path(sys.argv[2]).touch(); time.sleep(90)', str(library), str(ready)])
        self.extra_processes.append(process)
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(ready.exists())
        self.env['MOCK_PORT'] = ('UNCONN 0 0 0.0.0.0:8888 *:* users:(("MicroXRCEAgent",pid='
                                 + str(process.pid) + ',fd=5))')

    def test_external_agent_is_reused(self):
        self.external_agent_fixture('2.4.2')
        self.run_script('start.sh', '--no-gui')
        run = next(call for call in self.calls() if call[0] == 'run')
        self.assertIn('PJ_START_AGENT=0', run)
        self.run_script('stop.sh')
        self.assertIsNone(self.sim.poll())

    def test_incompatible_agent_v3_is_not_reused_or_stopped(self):
        self.external_agent_fixture('3.0.1')
        result = self.run_script('start.sh', '--no-gui', success=False)
        self.assertIn('Incompatible external Agent version', result.stderr)
        self.assertFalse(any(call[0] in ('run', 'rm', 'stop') for call in self.calls()))
        self.assertIsNone(self.extra_processes[0].poll())

    def test_modes_are_mutually_exclusive(self):
        result = self.run_script('start.sh', '--attach', '--launch', success=False)
        self.assertIn('mutually exclusive', result.stderr)
        self.assertFalse((self.root / 'docker.calls').exists())

    def test_repeat_start_detects_stopped_px4(self):
        self.run_script('start.sh', '--no-gui')
        self.sim.terminate()
        self.sim.wait(timeout=5)
        result = self.run_script('start.sh', '--no-gui', success=False)
        self.assertIn('selected PX4 has stopped', result.stderr)
        self.assertEqual(sum(call[0] == 'run' for call in self.calls()), 1)

    def test_repeat_start_rechecks_actual_reception(self):
        self.run_script('start.sh', '--no-gui')
        self.env['MOCK_PROBE_FAIL'] = '1'
        self.run_script('start.sh', '--no-gui', success=False)
        self.assertEqual(sum(call[0] == 'run' for call in self.calls()), 1)
        self.assertTrue(any(call[0] == 'exec' and call[-1] == 'probe' for call in self.calls()))

    def test_unknown_udp_owner_is_not_touched(self):
        self.env['MOCK_PORT'] = 'UNCONN 0 0 0.0.0.0:8888 *:* users:(("something_else",pid=12,fd=5))'
        result = self.run_script('start.sh', '--no-gui', success=False)
        self.assertIn('occupied by an unknown process', result.stderr)
        self.assertFalse(any(call[0] in ('run', 'rm', 'stop') for call in self.calls()))

    def test_foreign_container_is_not_stopped(self):
        (self.root / 'docker.json').write_text(json.dumps(dict(id='b' * 64, owner='foreign', running=True)))
        self.run_script('stop.sh', success=False)
        self.assertFalse(any(call[0] in ('rm', 'stop') for call in self.calls()))

    def test_start_failure_removes_only_new_container(self):
        self.env['MOCK_START_FAIL'] = '1'
        self.run_script('start.sh', '--no-gui', success=False)
        removals = [call for call in self.calls() if call[0] == 'rm']
        self.assertEqual(removals, [['rm', '--force', 'a' * 64]])
        self.assertIsNone(self.sim.poll())

    def test_docker_start_failure_without_id_cleans_only_its_attempt(self):
        self.env['MOCK_RUN_FAIL_NO_ID'] = '1'
        self.run_script('start.sh', '--no-gui', success=False)
        removals = [call for call in self.calls() if call[0] == 'rm']
        self.assertEqual(removals, [['rm', '--force', 'a' * 64]])
        self.assertIsNone(self.sim.poll())

    def test_docker_start_failure_does_not_remove_another_launch(self):
        self.env['MOCK_RUN_FAIL_NO_ID'] = '1'
        self.env['MOCK_FOREIGN_LAUNCH'] = '1'
        self.run_script('start.sh', '--no-gui', success=False)
        self.assertFalse(any(call[0] == 'rm' for call in self.calls()))
        self.assertIsNone(self.sim.poll())

    def test_doctor_checks_combined_data_and_gui_status(self):
        self.run_script('start.sh', '--no-gui')
        self.run_script('doctor.sh')
        self.assertTrue(any(call[0] == 'exec' and call[-1] == 'diagnose' for call in self.calls()))
        self.env['MOCK_DIAGNOSE_FAIL'] = '1'
        self.run_script('doctor.sh', success=False)

    def test_doctor_does_not_hide_an_exited_monitoring_container(self):
        self.run_script('start.sh', '--no-gui')
        state_file = self.root / 'docker.json'
        state = json.loads(state_file.read_text())
        state['running'] = False
        state_file.write_text(json.dumps(state))
        result = self.run_script('doctor.sh', success=False)
        self.assertIn('monitoring container has exited', result.stderr)
        self.assertEqual(sum(call[0] == 'run' for call in self.calls()), 1)

    def test_domain_mismatch_is_not_silently_changed(self):
        self.env['MOCK_DOMAIN'] = '2'
        result = self.run_script('start.sh', '--no-gui', success=False)
        self.assertIn('Domain ID differs', result.stderr)
        self.assertFalse(any(call[0] == 'run' for call in self.calls()))

    def test_running_previous_binary_is_recognized_after_rebuild(self):
        binary = self.repo / 'build/px4_sitl_default/bin/px4'
        binary.unlink()
        shutil.copy2('/bin/sleep', binary)
        result = self.run_script('start.sh', '--no-gui')
        self.assertIn('predates the latest build', result.stdout)
        self.assertIsNone(self.sim.poll())

    def test_px4_cli_symlinks_are_not_counted_as_main_instances(self):
        helper = self.repo / 'build/px4_sitl_default/bin/px4-testhelper'
        helper.symlink_to('px4')
        self.extra_processes.append(subprocess.Popen([str(helper), '90']))
        self.extra_processes.append(subprocess.Popen([str(helper), '90']))
        self.run_script('start.sh', '--no-gui')
        runtime = json.loads((self.state / 'runtime.json').read_text())
        self.assertEqual(runtime['px4_pid'], str(self.sim.pid))
        self.assertTrue(all(process.poll() is None for process in self.extra_processes))

    def test_multiple_main_px4_processes_are_still_rejected(self):
        binary = self.repo / 'build/px4_sitl_default/bin/px4'
        self.extra_processes.append(subprocess.Popen([str(binary), '90']))
        result = self.run_script('start.sh', '--no-gui', success=False)
        self.assertIn('Several PX4 instances', result.stderr)
        self.assertFalse(any(call[0] == 'run' for call in self.calls()))

    def test_matching_uninspectable_px4_refuses_duplicate_launch(self):
        hooks = self.root / 'python-hooks'
        hooks.mkdir()
        (hooks / 'sitecustomize.py').write_text(
            'import os\noriginal = os.readlink\n'
            'def guarded(path, *args, **kwargs):\n'
            '    if os.fspath(path) == "/proc/" + os.environ["DENY_EXE_PID"] + "/exe":\n'
            '        raise PermissionError("simulated primary-group restriction")\n'
            '    return original(path, *args, **kwargs)\n'
            'os.readlink = guarded\n')
        self.env['PYTHONPATH'] = str(hooks)
        self.env['DENY_EXE_PID'] = str(self.sim.pid)
        result = self.run_script('start.sh', '--launch', '--no-gui', success=False)
        self.assertIn('Cannot verify existing process PID', result.stderr)
        self.assertIn('fresh login session', result.stderr)
        self.assertFalse(any(call[0] == 'run' for call in self.calls()))
        self.assertIsNone(self.sim.poll())

    def test_stop_without_session_is_idempotent(self):
        self.run_script('stop.sh')
        self.assertFalse(any(call[0] in ('rm', 'stop') for call in self.calls()))


if __name__ == '__main__':
    unittest.main()
