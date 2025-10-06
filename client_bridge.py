#!/usr/bin/env python3
# client_bridge.py
import os
import asyncio
import threading
import time
from asyncua import Client, ua

OPCUA_URL = os.environ.get("OPCUA_URL", "opc.tcp://192.168.97.228:4840/freeopcua/server/")

# Try to import rclpy; if not present we fallback to printing
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import UInt16, String
    from geometry_msgs.msg import PoseStamped
    HAS_ROS = True
except Exception:
    HAS_ROS = False
    # Create a minimal Node-like stand-in for logging if ROS not available
    class DummyNode:
        def __init__(self, name):
            self.name = name
        def get_logger(self):
            class L:
                @staticmethod
                def info(m): print("[INFO]", m)
                @staticmethod
                def warning(m): print("[WARN]", m)
                @staticmethod
                def error(m): print("[ERR]", m)
            return L()
    Node = DummyNode

class OpcuaBridge(Node):
    def __init__(self):
        Node.__init__(self, "opcua_bridge") if HAS_ROS else Node("opcua_bridge")
        if HAS_ROS:
            self.state_pub = self.create_publisher(String, "/plc/state", 10)
            self.fault_pub = self.create_publisher(UInt16, "/plc/faults", 10)
            self.mission_sub = self.create_subscription(UInt16, "/cmd/mission", self.on_mission, 10)
            self.goal_sub = self.create_subscription(PoseStamped, "/cmd/goal_pose", self.on_goal, 10)
        else:
            self.get_logger().info("ROS2 not available; running in fallback print mode.")

        self.client = Client(OPCUA_URL)
        self.loop = asyncio.get_event_loop()
        self.nodes = {}            # friendly name -> Node
        self.nodeid_to_name = {}   # nodeid string -> friendly name
        self.hb = False

        # heartbeat timer (200ms)
        if HAS_ROS:
            self.create_timer(0.2, self.toggle_heartbeat)
        else:
            # fallback simple timer using thread loop
            pass

    def on_mission(self, msg):
        if "MissionId" in self.nodes:
            self.loop.create_task(self.write_typed(self.nodes["MissionId"], int(msg.data), ua.VariantType.UInt16))
        else:
            self.get_logger().warning("MissionId not resolved. Ignoring mission msg.")

    def on_goal(self, msg):
        if all(k in self.nodes for k in ("GoalX","GoalY","GoalTheta")):
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            # For demo: use 0.0 for theta; compute real yaw if needed
            self.loop.create_task(self.write_typed(self.nodes["GoalX"], x, ua.VariantType.Double))
            self.loop.create_task(self.write_typed(self.nodes["GoalY"], y, ua.VariantType.Double))
            self.loop.create_task(self.write_typed(self.nodes["GoalTheta"], 0.0, ua.VariantType.Double))
        else:
            self.get_logger().warning("Goal nodes not resolved. Ignoring goal_pose.")

    def toggle_heartbeat(self):
        # called by ROS timer. If ROS not present, we'll toggle from async run
        self.hb = not self.hb
        if "Heartbeat" in self.nodes:
            self.loop.create_task(self.write_typed(self.nodes["Heartbeat"], bool(self.hb), ua.VariantType.Boolean))
        else:
            # log occasionally
            if int(time.time()) % 5 == 0:
                self.get_logger().warning("Heartbeat node not resolved yet.")

    async def write_typed(self, node, value, vartype):
        try:
            await node.write_value(ua.Variant(value, vartype))
        except Exception as e:
            self.get_logger().error(f"Failed to write {node}: {e}")

    async def discover_nodes(self, timeout_s=10.0):
        """ Browse Objects and populate self.nodes mapping. Retry for timeout_s seconds. """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                ns = await self.client.get_namespace_array()
                self.get_logger().info(f"Namespaces: {ns}")
                children = await self.client.nodes.objects.get_children()
                # print top children
                top = []
                for ch in children:
                    try:
                        bn = await ch.read_browse_name()
                        top.append((bn.Name, ch.nodeid.to_string()))
                    except Exception:
                        top.append(("<?>", ch.nodeid.to_string()))
                self.get_logger().info(f"Top-level Objects: {top}")

                # find App
                app_node = None
                for ch in children:
                    try:
                        bn = await ch.read_browse_name()
                        if bn.Name.lower() == "app":
                            app_node = ch
                            break
                    except Exception:
                        continue

                if not app_node:
                    self.get_logger().warning("App node not found yet; retrying...")
                    await asyncio.sleep(0.5)
                    continue

                # enumerate app children
                app_children = await app_node.get_children()
                for c in app_children:
                    try:
                        bn = await c.read_browse_name()
                        name = bn.Name
                        self.nodes[name] = c
                        self.nodeid_to_name[c.nodeid.to_string()] = name
                        try:
                            val = await c.read_value()
                        except Exception:
                            val = "<no-read>"
                        self.get_logger().info(f"Resolved {name} = {val} (nodeid={c.nodeid.to_string()})")
                    except Exception as e:
                        self.get_logger().warning(f"Could not read child: {e}")

                return True
            except Exception as e:
                self.get_logger().warning(f"Discovery error: {e}; retrying...")
                await asyncio.sleep(0.5)
        return False

    async def run_async(self):
        try:
            await self.client.connect()
            self.get_logger().info(f"Connected to OPC UA server @ {OPCUA_URL}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        ok = await self.discover_nodes(timeout_s=10.0)
        if not ok:
            self.get_logger().warning("Could not resolve App children in time; proceeding with what we have.")

        # subscribe to RobotBusy and FaultCode if present
        handler = SubHandler(self)
        sub = await self.client.create_subscription(200, handler)
        for name in ("RobotBusy","FaultCode"):
            if name in self.nodes:
                await sub.subscribe_data_change(self.nodes[name])
                self.get_logger().info(f"Subscribed to {name}")
            else:
                self.get_logger().warning(f"{name} not present; cannot subscribe")

        # If ROS not available, create our own heartbeat loop:
        if not HAS_ROS:
            async def hb_loop():
                while True:
                    self.hb = not self.hb
                    if "Heartbeat" in self.nodes:
                        await self.write_typed(self.nodes["Heartbeat"], bool(self.hb), ua.VariantType.Boolean)
                    await asyncio.sleep(0.2)
            asyncio.create_task(hb_loop())

        # keep alive until rclpy shutdown or pr
