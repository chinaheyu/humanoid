import moteus
import asyncio


async def main(args=None):
    transport_left = moteus.Fdcanusb('/dev/serial/by-id/usb-mjbots_fdcanusb_826543DB-if00')
    transport_right = moteus.Fdcanusb('/dev/serial/by-id/usb-mjbots_fdcanusb_1EB12734-if00')
    for i in range(14, 19):
        c = moteus.Controller(id=i, transport=transport_left)
        await c.set_stop()
    for i in range(19, 24):
        c = moteus.Controller(id=i, transport=transport_right)
        await c.set_stop()


if __name__ == '__main__':
    asyncio.run(main())
