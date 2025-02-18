# %%
import yaml
import argparse
import zmq
from zmq.asyncio import Context, Poller
import capnp
import asyncio
from typing import Any, Callable
import os
import time
import rospy
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct


# %%
# switch to the directory of the script
pwd = os.path.dirname(os.path.realpath(__file__))
os.chdir(pwd)

# ros messsage type name to class map
ros_type_map = {
    Float32MultiArray.__name__: Float32MultiArray,
    Imu.__name__: Imu,
    Image.__name__: Image,
    PointCloud2.__name__: PointCloud2,
}

# proto type name to class map
proto_type_map = {}

# sensor name to publisher function map
name_pubfunc_map = {}

# Read cfg.yml configuration file
with open(f"{pwd}/cfg.yml", "r") as file:
    config = yaml.safe_load(file)

recv_total_bytes = 0


# %%
def get_proto_class(proto_type: str) -> Any:
    global proto_type_map
    if proto_type in proto_type_map:
        return proto_type_map[proto_type]
    else:
        proto_class = capnp.load(f"{pwd}/messages/{proto_type}.capnp").__dict__[
            proto_type
        ]
        proto_type_map[proto_type] = proto_class
        return proto_class


# %%
# proto to messages
def Status_to_msg(data: dict) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [
        data["timestamp"],
        data["cpuUsage"],
        data["cpuTemp"],
        data["memUsage"],
        data["totalReadBytes"],
        data["batteryVoltage"],
        data["batteryCurrent"],
    ]
    return msg


def Imu_to_msg(data: dict):
    msg = Imu()
    msg.header.stamp = rospy.Time.from_sec(data["timestamp"] * 1e-9)
    msg.header.frame_id = data["frame_id"]
    msg.angular_velocity.x = data["angularVelocity"]["x"]
    msg.angular_velocity.y = data["angularVelocity"]["y"]
    msg.angular_velocity.z = data["angularVelocity"]["z"]
    msg.linear_acceleration.x = data["linearAcceleration"]["x"]
    msg.linear_acceleration.y = data["linearAcceleration"]["y"]
    msg.linear_acceleration.z = data["linearAcceleration"]["z"]
    return msg


def Image_to_msg(data: dict):
    bridge = CvBridge()
    img = cv2.imdecode(
        np.frombuffer(bytes(data["data"]), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    msg.header.stamp = rospy.Time.from_sec(data["timestamp"] * 1e-9)
    msg.header.frame_id = data["frame_id"]
    return msg


def PointCloud_to_msg(data: dict):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    header = Header()
    header.stamp = rospy.Time.from_sec(data["timestamp"] * 1e-9)
    header.frame_id = data["frame_id"]
    rawdata = []
    for point in data["points"]:
        rawdata.append([point["x"], point["y"], point["z"], point["i"]])
    return point_cloud2.create_cloud(header, fields, rawdata)


# %%
# create publisher funcs for each sensor
def create_pub_func(
    sensor_name: str, to_msg: Callable[[dict], Any]
) -> Callable[[dict], None]:
    ros_type = ros_type_map[config[sensor_name]["ros"]]
    topic = config[sensor_name]["topic"]
    publisher = rospy.Publisher(
        topic, ros_type, queue_size=10 * config[sensor_name]["rate"]
    )

    def pub_func(data: dict):
        msg = to_msg(data)
        publisher.publish(msg)

    return pub_func


def init_ros_publishers():
    global name_pubfunc_map
    for sensor_name in config["sensors"]:
        proto_name = config[sensor_name]["type"]
        name_pubfunc_map[sensor_name] = create_pub_func(
            sensor_name, globals()[f"{proto_name}_to_msg"]
        )


# %%
# sensor task class
class SensorTask:
    def __init__(self, name: str, ctx: zmq.asyncio.Context):
        self.name = name
        self.frame_id = "frame_id" in config[name] and config[name]["frame_id"]
        self.socket = ctx.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, name)
        self.socket.connect(config["app"]["address"])
        self.poller = Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.pub_func: Callable[[dict], None] = name_pubfunc_map[name]
        self.proto = get_proto_class(config[name]["type"])

    async def run(self):
        print(f"Subscribed to {self.name} from {config['app']['address']}")
        global recv_total_bytes
        while not rospy.is_shutdown():
            if not await self.poller.poll(5):
                continue
            data = await self.socket.recv()
            recv_total_bytes += len(data)
            msg = self.proto.from_bytes_packed(data[len(self.name) :]).to_dict()
            if self.frame_id:
                msg["frame_id"] = self.frame_id
            self.pub_func(msg)
        print(f"Unsubscribed from {self.name}")


# %%
async def log_recv_speed():
    global recv_total_bytes
    while not rospy.is_shutdown():
        await asyncio.sleep(1)
        print(f"Received {recv_total_bytes * 8 * 1e-6:.2f} Mb/s", end="\r")
        recv_total_bytes = 0
    print("")


async def main():
    rospy.init_node("tinysk", anonymous=True)
    init_ros_publishers()

    with Context() as context:
        tasks = [SensorTask(name, context).run() for name in config["sensors"]]
        tasks.append(log_recv_speed())
        await asyncio.gather(*tasks)


if __name__ == "__main__":
    asyncio.run(main())
