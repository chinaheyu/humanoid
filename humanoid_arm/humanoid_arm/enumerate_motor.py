import asyncio
import moteus


async def detect_motor(motor_id):
    c = moteus.Controller(motor_id)
    s = moteus.Stream(c)

    try:
        response = await asyncio.wait_for(s.command(b'conf get id.id', allow_any_response=True), 0.1)
    except asyncio.exceptions.TimeoutError:
        return None
    return response.decode('utf8')


async def enumerate_motor(max_id):
    result = []
    for i in range(1, max_id):
        r = await detect_motor(i)
        if r is not None:
            result.append(i)
            print(f'motor {i} response {r}')
    return result


async def main():
    motor_id = await enumerate_motor(30)
    print(motor_id)


if __name__ == '__main__':
    asyncio.run(main())
