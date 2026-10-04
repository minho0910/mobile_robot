"""Hash ROS message values while ignoring nondeterministic CDR padding bytes."""

import json

from rosidl_runtime_py.convert import message_to_ordereddict


def update_digest(digest, name, timestamp_ns, message):
    digest.update(name.encode("utf-8") + b"\0" + timestamp_ns.to_bytes(8, "little"))
    if name == "/points_raw":
        metadata = {
            "header": message_to_ordereddict(message.header),
            "height": message.height,
            "width": message.width,
            "fields": [message_to_ordereddict(field) for field in message.fields],
            "is_bigendian": message.is_bigendian,
            "point_step": message.point_step,
            "row_step": message.row_step,
            "is_dense": message.is_dense,
        }
        digest.update(json.dumps(metadata, sort_keys=True, separators=(",", ":")).encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes(message.data))
    else:
        digest.update(
            json.dumps(message_to_ordereddict(message), sort_keys=True, separators=(",", ":")).encode("utf-8")
        )
    digest.update(b"\0")
