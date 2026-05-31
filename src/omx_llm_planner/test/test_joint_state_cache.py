import time
from sensor_msgs.msg import JointState
from omx_llm_planner.joint_state_cache import JointStateCache


def test_returns_position_for_named_joint():
    c = JointStateCache(max_age_sec=10.0, clock_now=lambda: 100.0)
    msg = JointState(); msg.name = ["joint1", "joint2"]; msg.position = [0.5, 1.0]
    c.update(msg, stamp=100.0)
    assert c.get("joint1") == 0.5


def test_stale_returns_none():
    c = JointStateCache(max_age_sec=1.0, clock_now=lambda: 105.0)
    msg = JointState(); msg.name = ["joint1"]; msg.position = [0.3]
    c.update(msg, stamp=100.0)            # 5초 전 -> stale
    assert c.get("joint1") is None


def test_missing_joint_returns_none():
    c = JointStateCache(max_age_sec=10.0, clock_now=lambda: 100.0)
    msg = JointState(); msg.name = ["joint2"]; msg.position = [1.0]
    c.update(msg, stamp=100.0)
    assert c.get("joint1") is None
