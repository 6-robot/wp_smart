"""Exercise installer backup and rollback in a sandbox with mocked udev calls."""

from pathlib import Path
import shlex
import subprocess

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts' / 'install_udev_rule.sh'
RULE = PACKAGE / 'udev' / 'smart_base.rules'


def run_installer_fixture(tmp_path, *, failure='', old_target=True):
    rules = tmp_path / 'rules with spaces'
    rules.mkdir()
    target = rules / '99-smart-base.rules'
    if old_target:
        target.write_text('# previous chassis rule\n')
    (rules / 'legacy.rules').write_text('SUBSYSTEM=="tty", SYMLINK += "smart_base"\n')
    (rules / 'unrelated.rules').write_text(
        'SUBSYSTEM=="tty", SYMLINK+="ydlidar" # SYMLINK+="smart_base"\n')
    alias = tmp_path / 'smart_base'
    calls = tmp_path / 'udev_calls'
    code = f'''
source {shlex.quote(str(SCRIPT))}
rule_source={shlex.quote(str(RULE))}
rules_dir={shlex.quote(str(rules))}
target="${{rules_dir}}/99-smart-base.rules"
alias_path={shlex.quote(str(alias))}
base_device=/dev/null
failure={shlex.quote(failure)}
calls={shlex.quote(str(calls))}
failed_once=false
udevadm() {{
  echo "$*" >> "${{calls}}"
  if [[ $1 == control && $failure == reload && $failed_once == false ]]; then
    failed_once=true
    return 1
  fi
  if [[ $1 == trigger && $failure != verify ]]; then
    ln -sfn /dev/null "${{alias_path}}"
  fi
}}
find_conflicting_rules
apply_rule
'''
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True, timeout=10)
    return result, rules, target, alias, calls


def test_install_keeps_backup_and_only_disables_conflicting_alias(tmp_path):
    result, rules, target, alias, calls = run_installer_fixture(tmp_path)
    assert result.returncode == 0, result.stderr
    assert target.read_bytes() == RULE.read_bytes()
    assert next(rules.glob('99-smart-base.rules.bak.*')).read_text() == '# previous chassis rule\n'
    assert not (rules / 'legacy.rules').exists()
    assert len(list(rules.glob('legacy.rules.disabled.*'))) == 1
    assert 'ydlidar' in (rules / 'unrelated.rules').read_text()
    assert alias.resolve() == Path('/dev/null')
    assert '--sysname-match=null' in calls.read_text()
    assert 'settle --timeout=10' in calls.read_text()


@pytest.mark.parametrize('failure', ['reload', 'verify'])
def test_failed_install_restores_original_target_and_conflicting_rule(tmp_path, failure):
    result, rules, target, _, _ = run_installer_fixture(tmp_path, failure=failure)
    assert result.returncode != 0
    assert 'restoring previous' in result.stderr
    assert target.read_text() == '# previous chassis rule\n'
    assert (rules / 'legacy.rules').exists()
    assert not list(rules.glob('legacy.rules.disabled.*'))
    assert 'ydlidar' in (rules / 'unrelated.rules').read_text()


def test_failed_first_install_does_not_leave_a_new_active_rule(tmp_path):
    result, rules, target, _, _ = run_installer_fixture(
        tmp_path, failure='verify', old_target=False)
    assert result.returncode != 0
    assert not target.exists()
    assert (rules / 'legacy.rules').exists()


def test_wrong_adapter_identity_is_rejected_before_installation():
    code = f'''
source {shlex.quote(str(SCRIPT))}
read_rule_identity
requested_device=/dev/null
udevadm() {{
  echo 'ID_VENDOR_ID=1a86'
  echo 'ID_MODEL_ID=55d4'
  echo 'ID_SERIAL_SHORT=some_other_adapter'
}}
find_base_device
'''
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert 'found 0' in result.stderr
