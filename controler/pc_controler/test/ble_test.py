# import asyncio
# from bleak import BleakScanner

# async def main():
#     devices = await BleakScanner.discover()
#     for d in devices:
#         if d.name == "HC-08":
#             print(d)
#             print(d.details)
#             print(d.metadata)

# asyncio.run(main())

#==============================================================================

# UUIDs = ['00001800-0000-1000-8000-00805f9b34fb', '00001801-0000-1000-8000-00805f9b34fb', '0000180a-0000-1000-8000-00805f9b34fb', '0000ffe0-0000-1000-8000-00805f9b34fb', '0000fff0-0000-1000-8000-00805f9b34fb']

# Services:
# 0000ffe0-0000-1000-8000-00805f9b34fb (Handle: 52): Vendor specific
# 0000180a-0000-1000-8000-00805f9b34fb (Handle: 16): Device Information
# 00001801-0000-1000-8000-00805f9b34fb (Handle: 12): Generic Attribute Profile
# 0000fff0-0000-1000-8000-00805f9b34fb (Handle: 35): Vendor specific

#==============================================================================

# import asyncio
# from bleak import BleakClient, BleakScanner

# address = "94:A9:A8:31:E5:6B"

# async def main():
#     devices = await BleakScanner.discover()
#     for d in devices:
#         if d.name == 'HC-08':
#             print(d)
#             print(d.metadata)
#             print('Found it')

#     async with BleakClient(address) as client:
#         svcs = await client.get_services()
#         print("Services:")
#         for service in svcs:
#             print(service)

# asyncio.run(main())

#==============================================================================

# import asyncio
# from bleak import BleakClient

# address = "94:A9:A8:31:E5:6B"
# MODEL_NBR_UUID = "00002a24-0000-1000-8000-00805f9b34fb"

# async def main(address):
#     async with BleakClient(address) as client:
#         model_number = await client.read_gatt_char(MODEL_NBR_UUID)
#         print("Model Number: {0}".format("".join(map(chr, model_number))))

# asyncio.run(main(address))

#==============================================================================

import asyncio
from bleak import BleakClient, BleakScanner

address = "94:A9:A8:31:E5:6B"
# read_characteristic = "0000fff1-0000-1000-8000-00805f9b34fb"
# write_characteristic = "0000fff1-0000-1000-8000-00805f9b34fb"

# read_characteristic = "0000fff2-0000-1000-8000-00805f9b34fb"
# write_characteristic = "0000fff3-0000-1000-8000-00805f9b34fb"

read_characteristic = "0000fff5-0000-1000-8000-00805f9b34fb"
write_characteristic = "0000fff5-0000-1000-8000-00805f9b34fb"

write_characteristic = "0000ffe1-0000-1000-8000-00805f9b34fb"

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "HC-08":
            print(d)
            print(d.metadata)
            print("Found HC-08")

            async with BleakClient(address) as client:
                # await client.connect()
                if client.is_connected:
                    print("Connected to HC-08!")

                    bytes_to_send = bytearray(map(ord, "a"))
                    await client.write_gatt_char(write_characteristic, bytes_to_send)
                    print("Message sent!")

                    await asyncio.sleep(5.0)

                    await client.stop_notify(read_characteristic)
                    await client.disconnect()
        

asyncio.run(main())

#==============================================================================

# import asyncio
# from bleak import BleakClient

# adresse_mac = "94:A9:A8:31:E5:6B" # Remplacer avec l'adresse MAC de votre module
# service_uuid = "0000ffe0-0000-1000-8000-00805f9b34fb" # UUID du service HC-08

# async def connexion():
#     async with BleakClient(adresse_mac) as client:
#         services = await client.get_services()
#         for service in services:
#             if service.uuid == service_uuid:
#                 characteristics = service.characteristics
#                 for c in characteristics:
#                     # Envoyer les données à la caractéristique correspondante (à déterminer)
#                     donnees_a_envoyer = "a"
#                     await client.write_gatt_char(c.uuid, donnees_a_envoyer.encode())
#                     print(c)
                    
# loop = asyncio.get_event_loop()
# loop.run_until_complete(connexion())
