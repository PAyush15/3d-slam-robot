#!/usr/bin/env python3
import asyncio, os, threading
from asyncua import Client, ua
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, String, Bool
from geometry_msgs.msg import PoseStamped

SERVER_URL = os.environ.get("OPCUA_URL", "opc.tcp://192.168.1.50:4840/freeopcua/server/")  # set to your laptop IP

class Bridge(Node):
    def __init__(self):
        super().__init__("opcua_bridge")
        # ROS pubs/subs
        self.state_pub  = self.create_publisher(String, "/plc/state", 10)
        self.fault_pub  = self.create_publisher(UInt16, "/plc/faults", 10)

        self.mission_sub = self.create_subscription(UInt16, "/cmd/mission", self.on_mission, 10)
        self.goal_sub    = self.create_subscription(PoseStamped, "/cmd/goal_pose", self.on_goal, 10)

        # Heartbeat timer (PLC should watchdog this)
        self.hb = False
        self.create_timer(0.2, self.on_heartbeat)

        # Async OPC UA loop in background
        self.client = Client(SERVER_URL)
        self.loop = asyncio.get_event_loop()
        self.nodes = {}

    def on_mission(self, msg: UInt16):
        # write mission to UA
        self.loop.create_task(self.nodes["MissionId"].write_value(ua.UInt16(msg.data)))

    def on_goal(self, msg: PoseStamped):
        self.loop.create_task(self.nodes["GoalX"].write_value(ua.Double(msg.pose.position.x)))
        self.loop.create_task(self.nodes["GoalY"].write_value(ua.Double(msg.pose.position.y)))
        # theta from orientation (simplified: assume yaw in z for demo)
        # In real use: convert quaternion → yaw properly
        self.loop.create_task(self.nodes["GoalTheta"].write_value(ua.Double(0.0)))

    def on_heartbeat(self):
        self.hb = not self.hb
        self.loop.create_task(self.nodes["Heartbeat"].write_value(ua.Boolean(self.hb)))

    async def run_async(self):
        await self.client.connect()
        # Security example (when you enable it):
        # await self.client.set_security_string("Basic256Sha256,SignAndEncrypt,./client_cert.der,./client_key.pem,./server_cert.der")

        # Resolve nodes once
        root = self.client.nodes.root
        objects = self.client.nodes.objects
        # Browse path into our namespace
        app = await objects.get_child(["0:Objects", "2:App"])  # ns=2 from server.py
        get = lambda n: app.get_child([f"2:{n}"])
        self.nodes["MissionId"] = await get("MissionId")
        self.nodes["GoalX"]     = await get("GoalX")
        self.nodes["GoalY"]     = await get("GoalY")
        self.nodes["GoalTheta"] = await get("GoalTheta")
        self.nodes["Heartbeat"] = await get("Heartbeat")
        node_busy               = await get("RobotBusy")
        node_fault              = await get("FaultCode")

        # Create subscription for PLC→ROS updates
        handler = DataChangeHandler(self)
        sub = await self.client.create_subscription(100, handler)  # 100 ms publishing interval
        await sub.subscribe_data_change(node_busy)
        await sub.subscribe_data_change(node_fault)

        self.get_logger().info(f"Connected to OPC UA: {SERVER_URL}")
        while rclpy.ok():
            await asyncio.sleep(0.1)

class DataChangeHandler:
    def __init__(self, bridge: Bridge):
        self.bridge = bridge

    def datachange_notification(self, node, val, data):
        # Identify by browse name
        try:
            name = data.monitored_item.Value.SourceTimestamp  # not the name; we’ll actually resolve via node
        except:
            pass
        # A small helper: check nodeId’s Identifier string
        nid = node.nodeid.Identifier if hasattr(node.nodeid, "Identifier") else str(node)
        if "RobotBusy" in nid:
            self.bridge.state_pub.publish(String(data="BUSY" if val else "IDLE"))
        elif "FaultCode" in nid:
            self.bridge.fault_pub.publish(UInt16(data=int(val)))

def main():
    rclpy.init()
    bridge = Bridge()

    # Run asyncio in parallel with rclpy
    t = threading.Thread(target=lambda: asyncio.run(bridge.run_async()), daemon=True)
    t.start()
    try:
        rclpy.spin(bridge)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
