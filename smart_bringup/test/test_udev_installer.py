"""Check installation and error handling without changing system udev rules."""

from pathlib import Path
import shlex
import subprocess

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts' / 'create_udev_rule.sh'
RULE = PACKAGE / 'udev' / 'smart_base.rules'


@pytest.mark.parametrize('fail_reload', [False, True])
def test_install_and_stop_on_reload_failure(tmp_path, fail_reload):
    target = tmp_path / 'installed rule'
    calls = tmp_path / 'udev_calls'
    code = f'''

id() {{ echo 0; }}
install() {{ command install -m 0644 -- "$4" {shlex.quote(str(target))}; }}
udevadm() {{
  echo "$*" >> {shlex.quote(str(calls))}
  if [[ $1 == control && {shlex.quote(str(fail_reload))} == True ]]; then
    return 1
  fi
}}
export -f id install udevadm
bash {shlex.quote(str(SCRIPT))}
'''
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True, timeout=5)
    assert target.read_bytes() == RULE.read_bytes()
    assert target.stat().st_mode & 0o777 == 0o644
    operations = calls.read_text().splitlines()
    if fail_reload:
        assert result.returncode != 0
        assert operations == ['control --reload-rules']
        assert 'Installed chassis' not in result.stdout
    else:
        assert result.returncode == 0, result.stderr
        assert operations == [
            'control --reload-rules',
            'trigger --action=change --subsystem-match=tty',
            'settle --timeout=10',
        ]
