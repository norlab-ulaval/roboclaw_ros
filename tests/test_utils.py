from roboclaw_ros.utils import ROBOCLAW_ERRORS, decipher_rclaw_status
import pytest


@pytest.fixture
def marmotte_unit_statuses():
    value = 0x40068001
    return decipher_rclaw_status(value)


def test_marmotte_stat_len(marmotte_unit_statuses):
    assert len(marmotte_unit_statuses) == 3


def test_marmotte_stat_estop(marmotte_unit_statuses):
    assert ROBOCLAW_ERRORS[0x000001] in marmotte_unit_statuses


def test_marmotte_stat_overcurrent_warn(marmotte_unit_statuses):
    assert ROBOCLAW_ERRORS[0x020000] in marmotte_unit_statuses


def test_marmotte_stat_overvoltage_warn(marmotte_unit_statuses):
    assert ROBOCLAW_ERRORS[0x040000] in marmotte_unit_statuses
