import moteus
import asyncio
from itertools import chain
import time


async def cycle(controls):
    for c in controls:
        try:
            await asyncio.wait_for(c.set_stop(query=True), 0.2)
        except asyncio.exceptions.TimeoutError:
            print(f'Send command timeout, id: {c.id}.')


async def main(args=None):
    transport_left = moteus.Fdcanusb('/dev/serial/by-id/usb-mjbots_fdcanusb_826543DB-if00')
    transport_right = moteus.Fdcanusb('/dev/serial/by-id/usb-mjbots_fdcanusb_1EB12734-if00')
    control_left = [moteus.Controller(id=i, transport=transport_left) for i in range(14, 19)]
    control_right = [moteus.Controller(id=i, transport=transport_right) for i in range(19, 24)]
    while True:
        st = time.time()
        await asyncio.gather(cycle(control_left), cycle(control_right))
        dt = 1000 * (time.time() - st)
        if dt > 20:
            print(f'Cycle time: {dt} ms > 20 ms.')


if __name__ == '__main__':
    asyncio.run(main())
