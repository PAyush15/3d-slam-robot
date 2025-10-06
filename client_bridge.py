#!/usr/bin/env python3
# client_bridge.py  -- robust bridge that correctly schedules writes from ROS callbacks
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
    class DummyLogger:
        @staticmethod
        def info(m): print("[INFO]", m)
        @staticmethod
        def warning(m): print("[WARN]", m)
        @staticmethod
        def error(m): print("[ERR]", m)
    class DummyNode:
        def __init__(self, name): self._log = DummyLogger()
        def get_logger(self): return self._log
        # minimal stubs for methods used elsewhere
        def create_timer(self, *_): pass
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
        # async_loop will be set to the running asyncio loop inside run_async()
        self.async_loop = None

        self.nodes = {}            # friendly name -> Node
        self.nodeid_to_name = {}   # nodeid string -> friendly name
        self.hb = False

        # heartbeat call (if ROS present we use its timer)
        if HAS_ROS:
            self.create_timer(0.2, self.toggle_heartbeat)

    # ---- ROS callbacks ----
    def on_mission(self, msg):
        # called in ROS thread (main thread). Schedule UA write safely into UA asyncio loop.
        self.get_logger().info(f"Received ROS /cmd/mission -> {getattr(msg,'data',None)}")
        if "MissionId" not in self.nodes:
            self.get_logger().warning("MissionId node not resolved; cannot write mission.")
            return
        try:
            # schedule the coroutine thread-safely into the UA loop
            coro = self.write_typed(self.nodes["MissionId"], int(msg.data), ua.VariantType.UInt16)
            if self.async_loop is None:
                self.get_logger().error("Async loop not set yet; cannot schedule write.")
            else:
                asyncio.run_coroutine_threadsafe(coro, self.async_loop)
                self.get_logger().info("Scheduled write to MissionId")
        except Exception as e:
            self.get_logger().error(f"Exception scheduling Mission write: {e}")

    def on_goal(self, msg):
        self.get_logger().info("Received ROS /cmd/goal_pose")
        if not all(k in self.nodes for k in ("GoalX","GoalY","GoalTheta")):
            self.get_logger().warning("Goal nodes not resolved; ignoring goal")
            return
        # compute pose values
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        theta = 0.0
        # schedule writes
        if self.async_loop:
            asyncio.run_coroutine_threadsafe(self.write_typed(self.nodes["GoalX"], x, ua.VariantType.Double), self.async_loop)
            asyncio.run_coroutine_threadsafe(self.write_typed(self.nodes["GoalY"], y, ua.VariantType.Double), self.async_loop)
            asyncio.run_coroutine_threadsafe(self.write_typed(self.nodes["GoalTheta"], theta, ua.VariantType.Double), self.async_loop)
            self.get_logger().info(f"Scheduled goal write x={x} y={y} th={theta}")
        else:
            self.get_logger().error("Async loop not set; cannot write goal")

    def toggle_heartbeat(self):
        self.hb = not self.hb
        if "Heartbeat" in self.nodes and self.async_loop:
            asyncio.run_coroutine_threadsafe(self.write_typed(self.nodes["Heartbeat"], bool(self.hb), ua.VariantType.Boolean), self.async_loop)
        else:
            # occasional log if heartbeat node missing
            if int(time.time()) % 5 == 0:
                self.get_logger().warning("Heartbeat node not resolved (toggle_heartbeat)")

    # ---- OPC UA helpers ----
    async def write_typed(self, node, value, vartype):
        try:
            await node.write_value(ua.Variant(value, vartype))
            self.get_logger().info(f"Wrote {value} as {vartype.name} to node {self.nodeid_to_name.get(node.nodeid.to_string(), node.nodeid.to_string())}")
        except Exception as e:
            self.get_logger().error(f"Failed to write node {node}: {e}")

    async def discover_nodes(self, timeout_s=10.0):
        """ Browse Objects and populate self.nodes mapping. """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                ns = await self.client.get_namespace_array()
                self.get_logger().info(f"Namespaces: {ns}")
                children = await self.client.nodes.objects.get_children()
                top = []
                for ch in children:
                    try:
                        bn = await ch.read_browse_name()
                        top.append((bn.Name, ch.nodeid.to_string()))
                    except Exception:
                        top.append(("<?>", ch.nodeid.to_string()))
                self.get_logger().info(f"Top-level Objects: {top}")

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
                    self.get_logger().warning("App node not found; retrying...")
                    await asyncio.sleep(0.5)
                    continue

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
        """ This runs inside the asyncio thread. We capture the running loop here. """
        try:
            await self.client.connect()
            self.get_logger().info(f"Connected to OPC UA server @ {OPCUA_URL}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        # capture the running loop so other threads can schedule coroutines into it
        self.async_loop = asyncio.get_running_loop()
        self.get_logger().info("Captured asyncio loop for scheduling.")

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

        # If ROS not available, start a heartbeat loop here
        if not HAS_ROS:
            async def hb_loop():
                while True:
                    self.hb = not self.hb
                    if "Heartbeat" in self.nodes:
                        await self.write_typed(self.nodes["Heartbeat"], bool(self.hb), ua.VariantType.Boolean)
                    await asyncio.sleep(0.2)
            asyncio.create_task(hb_loop())

        try:
            while True:
                await asyncio.sleep(0.1)
        finally:
            await sub.delete()

class SubHandler:
    def __init__(self, bridge):
        self.bridge = bridge
    def datachange_notification(self, node, val, data):
        nid = node.nodeid.to_string()
        name = self.bridge.nodeid_to_name.get(nid, nid)
        if name == "RobotBusy":
            if HAS_ROS:
                msg = String()
                msg.data = "BUSY" if val else "IDLE"
                self.bridge.state_pub.publish(msg)
            else:
                self.bridge.get_logger().info(f"RobotBusy = {val}")
        elif name == "FaultCode":
            if HAS_ROS:
                m = UInt16()
                m.data = int(val)
                self.bridge.fault_pub.publish(m)
            else:
                self.bridge.get_logger().info(f"FaultCode = {val}")

def main():
    if HAS_ROS:
        rclpy.init()
        node = OpcuaBridge()
    else:
        node = OpcuaBridge()

    t = threading.Thread(target=lambda: asyncio.run(node.run_async()), daemon=True)
    t.start()

    if HAS_ROS:
        try:
            rclpy.spin(node)
        finally:
            rclpy.shutdown()
    else:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Exiting")

if __name__ == "__main__":
    main()
