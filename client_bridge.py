# opcua_browse.py
import asyncio
from asyncua import Client

URL = "opc.tcp://192.168.224.56:4840/freeopcua/server/"   # adjust if needed

async def main():
    async with Client(URL) as client:
        ns = await client.get_namespace_array()
        print("Namespaces:", ns)
        root = client.nodes.root
        objects = client.nodes.objects
        children = await objects.get_children()
        print("Objects children count:", len(children))
        for ch in children:
            bn = await ch.get_browse_name()
            print("Child browse name:", bn.Name, "nodeid:", ch.nodeid.to_string())

        # Optionally expand the 'App' object if present
        for ch in children:
            bn = await ch.get_browse_name()
            if bn.Name.lower() == "app":
                print("Found App node, listing its children:")
                subch = await ch.get_children()
                for s in subch:
                    sbn = await s.get_browse_name()
                    val = await s.read_value()
                    print("  ", sbn.Name, " => ", val, " nodeid:", s.nodeid.to_string())

asyncio.run(main())
