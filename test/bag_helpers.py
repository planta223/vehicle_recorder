"""Offline test adapter; never imported by the recorder process."""
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def read_bag(directory):
    rows = []
    for database in sorted((directory / 'bag').glob('*.db3')):
        with sqlite3.connect(str(database)) as db:
            for topic, type_name, data in db.execute(
                    'SELECT t.name,t.type,m.data FROM messages m JOIN topics t ON t.id=m.topic_id ORDER BY m.id'):
                rows.append(dict(topic=topic, data=message_to_ordereddict(
                    deserialize_message(data, get_message(type_name)))))
    return rows


def bag_bytes(directory):
    return {p.name: p.read_bytes() for p in (directory / 'bag').iterdir() if p.is_file()}
