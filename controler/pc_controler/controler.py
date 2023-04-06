import asyncio
from bleak import BleakClient, BleakScanner
import pygame
pygame.init()

async def mainLoop(client):

    win = pygame.display.set_mode((400, 400))
    pygame.display.set_caption("RC CAR")

    keyboard_input = ""

    run = True
    while run:

        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                run=False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            run=False
        if keys[pygame.K_UP]:
            print("KUP")
            keyboard_input = "f"
        if keys[pygame.K_DOWN]:
            keyboard_input = "b"
        if keys[pygame.K_LEFT]:
            keyboard_input = "l"
        if keys[pygame.K_RIGHT]:
            keyboard_input = "r"

        if keyboard_input != "":
            bytes_to_send = bytearray(map(ord, keyboard_input))
            keyboard_input = ""
            await client.write_gatt_char(write_characteristic, bytes_to_send)
            print("Message sent!")

        pygame.display.update()
        pygame.time.delay(100)


async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "HC-08":
            print("Found HC-08")

            async with BleakClient(address) as client:
                # await client.connect()
                if client.is_connected:
                    print("Connected to HC-08!")

                    await mainLoop(client)

                    pygame.quit()
                    await asyncio.sleep(2.0)

                    await client.stop_notify(read_characteristic)
                    await client.disconnect()


if __name__ == "__main__":

    address = "94:A9:A8:31:E5:6B"
    read_characteristic = "0000fff1-0000-1000-8000-00805f9b34fb"
    write_characteristic = "0000ffe1-0000-1000-8000-00805f9b34fb"

    asyncio.run(main())