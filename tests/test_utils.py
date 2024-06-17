from roboclaw_node_ros.utils import decipher_rclaw_status


def test_byte_unpacking():
    value = 491200
    statuses = decipher_rclaw_status(value)
    print(statuses)
    # TODO: Add unit cases for assertions
