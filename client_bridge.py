#!/usr/bin/env python3
import asyncio, os, threading, time
from asyncua import Client, ua
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, String, Bool
from geometry_msgs.msg import PoseStamped

SERVER_URL = os.environ.get("OPCUA_URL", "opc.tcp://192.168.97.228:4840/freeopcua/server/")

class Bridge(Node):
    def __init__(self):
        super().__init__("opcua_bridge")
        self.state_pub  = self.create_publisher(String, "/plc/state", 10)
        self.fault_pub  = self.create_publisher(UInt16, "/plc/faults", 10)

        self.mission_sub = self.create_subscription(UInt16, "/cmd/mission", self.on_mission, 10)
        self.goal_sub    = self.create_subscription(PoseStamped, "/cmd/goal_pose", self.on_goal, 10)

        self.hb = False
        self.create_timer(0.2, self.on_heartbeat)  # heartbeat publishes every 200 ms

        self.client = Client(SERVER_URL)
        self.loop = asyncio.get_event_loop()
        self.nodes = {}              # friendly name -> node object
        self.nodeid_to_name = {}     # nodeid string -> friendly name

    def on_mission(self, msg: UInt16):
        if "MissionId" in self.nodes:
            self.loop.create_task(self.safe_write(self.nodes["MissionId"], ua.UInt16(msg.data)))
        else:
            self.get_logger().warning("MissionId node not resolved yet; ignoring mission cmd")

    def on_goal(self, msg: PoseStamped):
        if all(k in self.nodes for k in ("GoalX","GoalY","GoalTheta")):
            self.loop.create_task(self.safe_write(self.nodes["GoalX"], ua.Double(msg.pose.position.x)))
            self.loop.create_task(self.safe_write(self.nodes["GoalY"], ua.Double(msg.pose.position.y)))
            # convert quaternion to yaw properly in production
            self.loop.create_task(self.safe_write(self.nodes["GoalTheta"], ua.Double(0.0)))
        else:
            self.get_logger().warning("Goal nodes not resolved yet; ignoring goal_pose")

    def on_heartbeat(self):
        # only try to write if we resolved the node
        if "Heartbeat" in self.nodes:
            self.hb = not self.hb
            self.loop.create_task(self.safe_write(self.nodes["Heartbeat"], ua.Boolean(self.hb)))
        else:
            # keep toggling locally but do not attempt write
            self.hb = not self.hb
            # Log only occasionally to avoid spam
            if int(time.time()) % 5 == 0:
                self.get_logger().warning("Heartbeat node not available yet; not writing")

    async def safe_write(self, node, val):
        try:
            await node.write_value(val)
        except Exception as e:
            self.get_logger().error(f"Failed writing to {node}: {e}")

    async def find_app_and_nodes(self):
        """
        Browse objects and find 'App' by browse name, then find child vars.
        Retry for a short time if not present yet.
        """
        objects = self.client.nodes.objects

        def _nodeid_str(node):
            try:
                return node.nodeid.to_string()
            except:
                return str(node)

        for attempt in range(20):   # ~10 seconds at 0.5s sleep
            try:
                # Print namespaces
                nsarr = await self.client.get_namespace_array()
                self.get_logger().info(f"Namespaces: {nsarr}")

                children = await objects.get_children()
                names = []
                for ch in children:
                    bn = await ch.read_browse_name()
                    names.append((bn.Name, _nodeid_str(ch)))
                self.get_logger().info(f"Objects children: {names}")

                # Find App child by name (case-insensitive)
                app_node = None
                for ch in children:
                    bn = await ch.read_browse_name()
                    if bn.Name.lower() == "app":
                        app_node = ch
                        break

                if app_node is None:
                    self.get_logger().warning("No 'App' object found yet on server; retrying...")
                    await asyncio.sleep(0.5)
                    continue

                # List App children and map them
                app_children = await app_node.get_children()
                for c in app_children:
                    bn = await c.read_browse_name()
                    bname = bn.Name
                    self.nodes[bname] = c
                    self.nodeid_to_name[c.nodeid.to_string()] = bname
                    try:
                        val = await c.read_value()
                    except Exception:
                        val = "<no-read>"
                    self.get_logger().info(f"Resolved node: {bname} = {val} (nodeid: {c.nodeid.to_string()})")

                # We found App and its children — break
                return True

            except Exception as e:
                self.get_logger().warning(f"Error during browse attempt {attempt}: {e}")
                await asyncio.sleep(0.5)

        return False

    async def run_async(self):
        # connect
        await self.client.connect()
        self.get_logger().info(f"Connected to OPC UA server @ {SERVER_URL}")

        ok = await self.find_app_and_nodes()
        if not ok:
            self.get_logger().error("Could not resolve App and its variables on the server after retries.")
            # still set up subscriptions for anything we have resolved
        # set up subscriptions for known nodes (RobotBusy, FaultCode)
        # build a map of node objects to friendly names for use in handler
        sub = await self.client.create_subscription(200, SubHandler(self))
        # subscribe what we can
        for name in ("RobotBusy", "FaultCode"):
            if name in self.nodes:
                await sub.subscribe_data_change(self.nodes[name])
                self.get_logger().info(f"Subscribed to {name}")
            else:
                self.get_logger().warning(f"{name} not available to subscribe")

        # keep running while rclpy is alive
        while rclpy.ok():
            await asyncio.sleep(0.1)

class SubHandler:
    def __init__(self, bridge: Bridge):
        self.bridge = bridge

    def datachange_notification(self, node, val, data):
        nid = node.nodeid.to_string()
        name = self.bridge.nodeid_to_name.get(nid, nid)
        if name == "RobotBusy":
            self.bridge.state_pub.publish(String(data="BUSY" if val else "IDLE"))
        elif name == "FaultCode":
            try:
                self.bridge.fault_pub.publish(UInt16(data=int(val)))
            except Exception:
                self.bridge.get_logger().error("FaultCode publish error")

def main():
    rclpy.init()
    bridge = Bridge()
    t = threading.Thread(target=lambda: asyncio.run(bridge.run_async()), daemon=True)
    t.start()
    try:
        rclpy.spin(bridge)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
