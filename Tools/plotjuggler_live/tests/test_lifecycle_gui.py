"""Test the exact GUI diagnostic embedded in the container management script."""

import contextlib
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


RUN_SCRIPT = Path(__file__).resolve().parents[1] / 'container/run.sh'
GUI_CODE = RUN_SCRIPT.read_text().split("<<'PY_GUI'\n", 1)[1].split('\nPY_GUI', 1)[0]


class GuiDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='px4-gui-diagnostic-')
        self.binary = Path(self.temporary.name) / 'plotjuggler'
        shutil.copy2('/bin/sleep', self.binary)
        self.process = None
        self.x11 = Mock()
        self.x11.XOpenDisplay.return_value = 1234
        self.x11.XCloseDisplay.return_value = 0

    def tearDown(self):
        if self.process is not None:
            self.process.terminate()
            self.process.wait(timeout=5)
        self.temporary.cleanup()

    def diagnose(self, enabled='1', display=':99'):
        output = io.StringIO()
        environment = {'PJ_GUI': enabled}
        if display is not None:
            environment['DISPLAY'] = display
        with patch.dict(os.environ, environment, clear=True), \
                patch.object(sys, 'argv', ['gui-diagnostic', str(self.binary)]), \
                patch('ctypes.CDLL', return_value=self.x11), contextlib.redirect_stdout(output):
            try:
                exec(compile(GUI_CODE, str(RUN_SCRIPT), 'exec'), {'__name__': '__main__'})
            except SystemExit as error:
                return error.code, output.getvalue()
        return 0, output.getvalue()

    def test_disabled_gui_does_not_require_display_or_process(self):
        result, output = self.diagnose(enabled='0', display=None)
        self.assertEqual(result, 0)
        self.assertIn('GUI: disabled', output)
        self.x11.XOpenDisplay.assert_not_called()

    def test_enabled_gui_requires_display(self):
        result, _ = self.diagnose(display=None)
        self.assertIn('DISPLAY is missing', result)

    def test_x11_authentication_failure_is_not_success(self):
        self.x11.XOpenDisplay.return_value = None
        result, _ = self.diagnose()
        self.assertIn('cannot connect', result)
        self.assertIn('Xauthority', result)

    def test_display_without_application_is_not_success(self):
        result, _ = self.diagnose()
        self.assertIn('application process is absent', result)

    def test_application_and_display_do_not_claim_visible_curves(self):
        self.process = subprocess.Popen([str(self.binary), '90'])
        result, output = self.diagnose()
        self.assertEqual(result, 0)
        self.assertIn(str(self.process.pid), output)
        self.assertIn('still require visual confirmation', output)


if __name__ == '__main__':
    unittest.main()
