import moteus
import asyncio


async def main(args=None):
    for i in range(14, 24):
        c = moteus.Controller(id=i)
        await c.set_stop()


if __name__ == '__main__':
    asyncio.run(main())
