#!/usr/bin/env python3
# test_browse_write.py
import asyncio
from asyncua import Client, ua

SERVER = "opc.tcp://192.168.97.228:4840/freeopcua/server/"

async def main():
    async with Client(SERVER) as client:
        print("Connected to", SERVER)
        children = await client.nodes.objects.get_children()
        print("Children of Objects:")
        for ch in children:
            try:
                bn = await ch.read_browse_name()
                print(" ", bn.Name, ch.nodeid.to_string())
            except Exception as e:
                print("  <err read_browse_name> node:", ch, e)

        # find App node
        app_node = None
        for ch in children:
            bn = await ch.read_browse_name()
            if bn.Name.lower() == "app":
                app_node = ch
                break

        if not app_node:
            print("App node not found")
            return

        print("App found:", app_node.nodeid.to_string())
        app_children = await app_node.get_children()
        for c in app_children:
            bn = await c.read_browse_name()
            try:
                val = await c.read_value()
            except Exception:
                val = "<no-read>"
            print("   ", bn.Name, "=", val, " nodeid:", c.nodeid.to_string())

        # write heartbeat if found
        hb = None
        for c in app_children:
            bn = await c.read_browse_name()
            if bn.Name.lower() == "heartbeat":
                hb = c
                break

        if hb:
            print("Writing heartbeat True")
            await hb.write_value(ua.Variant(True, ua.VariantType.Boolean))
            await asyncio.sleep(0.2)
            print("Writing heartbeat False")
            await hb.write_value(ua.Variant(False, ua.VariantType.Boolean))
            print("Done")
        else:
            print("No Heartbeat variable present under App")

if __name__ == "__main__":
    asyncio.run(main())
