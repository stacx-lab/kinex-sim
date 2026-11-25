"""Kinex's devices and world on a kinex-sim overlay: streams, requests, goals, and waits in each kind of world."""
import asyncio
import uuid

import pytest

pytest.importorskip('kinex.communication')
from kinex.communication import communication
from kinex.errors import OperationError, RPCError

SERVICE = {'transport': 'service', 'direction': 'input'}
LATCHED = {'transport': 'topic', 'direction': 'output', 'qos': {'depth': 1, 'durability': 'transient_local'}}


def world():
    """The operations every world declares, as this one serves them."""
    return {'world.now': {**SERVICE, 'endpoint': '/world/now', 'type': 'custom_interfaces/srv/Now'},
            'world.wait': {**SERVICE, 'endpoint': '/world/wait', 'type': 'custom_interfaces/srv/Wait'},
            'world.stop': {**SERVICE, 'endpoint': '/world/stop', 'type': 'std_srvs/srv/Trigger'}}


def finger():
    return {'finger.joint_state': {**LATCHED, 'endpoint': '/finger/joint_state', 'type': 'sensor_msgs/msg/JointState'},
            'finger.target': {'transport': 'topic', 'direction': 'input', 'endpoint': '/finger/target', 'type': 'std_msgs/msg/Float64'},
            'finger.stop': {**SERVICE, 'endpoint': '/finger/stop', 'type': 'std_srvs/srv/Trigger'}}


def arms():
    """The pair toy's two arms as Kinex operations: move goals and stops."""
    catalog = {}
    for arm in ('left', 'right'):
        catalog[f'{arm}.move'] = {'transport': 'action', 'direction': 'input', 'endpoint': f'/{arm}/move',
                                  'type': 'custom_interfaces/action/Json'}
        catalog[f'{arm}.stop'] = {**SERVICE, 'endpoint': f'/{arm}/stop', 'type': 'std_srvs/srv/Trigger'}
    return catalog


def run(graph, catalog, program):
    """Run program(io) in a Kinex scope on the graph's domain, then close the graph."""
    async def main():
        async with communication(catalog, domain_id=graph.lease.domain) as io:
            await program(io)
    try:
        asyncio.run(main())
    finally:
        graph.close()


def test_fanout_precision_bounded_queues_and_teardown():
    async def run():
        endpoint = '/test_' + uuid.uuid4().hex
        operation = {'transport': 'topic', 'direction': 'output', 'endpoint': endpoint,
                     'type': 'std_msgs/msg/Float64MultiArray', 'qos': {'depth': 2}}
        io = communication({'test.output': operation, 'test.input': {**operation, 'direction': 'input'}})
        async with io:
            streams = [io.subscribe(operation, 2) for _ in range(10)]
            command = io.device('test').topic('input')
            for index in range(30):
                await command.publish({'data': [1.2345678901234567, float(index)]})
                await asyncio.sleep(.02)  # Test producer pacing, not a workflow clock.
            await asyncio.sleep(.1)
            assert all(len(stream.buffer) <= 2 and len(stream.inbox) <= 2 for stream in streams)
            assert all(stream.latest.data[0] == 1.2345678901234567 for stream in streams)
            assert all(stream.dropped > 0 for stream in streams)
            streams[0].close()
            streams[0].close()
        assert not io.bridge.thread.is_alive()
        assert not io.bridge.context.ok()
    asyncio.run(run())


def test_a_device_reads_commands_and_requests_by_its_declared_operations(tmp_path):
    from tests.ros.support import Graph

    async def program(io):
        device, world = io.device('finger'), io.world
        assert set(device.operations) == {'joint_state', 'target', 'stop'}
        state = device.topic('joint_state')
        assert state is device.topic('joint_state') and state.latest is None
        assert list((await state.next()).position) == [0.]
        assert list((await state.next()).position) == [0.]  # a retained message is read again while paused
        await device.topic('target').publish({'data': -.5})
        assert await world.wait(.1) == pytest.approx(.1)
        assert list((await state.next()).position) == [-.5] and list(state.latest.position) == [-.5]
        assert (await device.service('stop')()).message == 'keeping the grip target'
        with pytest.raises(OperationError, match='seconds'):  # a service's rejection
            await io.device('world').service('wait')(seconds=float('nan'))
        io.catalog['finger.absent'] = {**SERVICE, 'endpoint': '/finger/absent', 'type': 'std_srvs/srv/Trigger'}
        with pytest.raises(TimeoutError):
            await device.service('absent')(timeout_s=.03)
        for wrong in (lambda: device.service('target'), lambda: device.topic('stop'), lambda: device.action('stop')):
            with pytest.raises(TypeError):
                wrong()
        with pytest.raises(TypeError, match='not a device output'):
            device.topic('target').next()
        with pytest.raises(RPCError, match='unknown or unavailable operation finger.missing'):
            device.topic('missing')
    run(Graph(tmp_path), {**world(), **finger()}, program)


def test_goals_run_in_parallel_and_awaiting_them_lets_the_world_run(tmp_path):
    """Kinex sends two arms' move goals and awaits them: sending one never steps the world, awaiting them does,
    to the step each ends at."""
    from tests.ros.support import Graph

    async def program(io):
        world, left, right = io.world, io.device('left').action('move'), io.device('right').action('move')

        async def time():
            return (await world.now()).time_s
        near, far = left({'x': .7, 'y': -.5}), right(x=1., y=-.5)  # 0.2 s and 0.5 s away
        assert near.status == 'sending' and await world.wait(0) == 0. and await time() == 0.
        assert await near == {'x': pytest.approx(.7)} and await time() == pytest.approx(.2)  # a Json action's object
        assert (near.status, far.status) == ('succeeded', 'executing')
        assert await asyncio.gather(near, far) == [{'x': pytest.approx(.7)}, {'x': pytest.approx(1.)}]
        assert await time() == pytest.approx(.5)
        # The latest feedback the device sent; its last may arrive after its result.
        assert far.feedback['x'] == pytest.approx(.95, abs=.06)
        assert await world.wait(.5) == pytest.approx(1.)  # a plain wait

        async def side(arm, *poses):
            """An arm through its poses in turn: a sequence of its own, as the other arm runs its."""
            return [(await arm(x=x, y=y), await time())[1] for x, y in poses]
        ends = await asyncio.gather(side(left, (.9, -.5), (.7, -.5)), side(right, (.9, -.5)))
        assert ends == [pytest.approx([1.2, 1.4]), pytest.approx([1.1])]
        slow = left(x=1.1, y=-.5)  # 0.4 s away
        # The goal or 0.2 s, whichever is first: the wait that lost is cancelled, or the world would run on for it.
        _, pending = await asyncio.wait({asyncio.ensure_future(slow._ended()), asyncio.ensure_future(world.wait(.2))},
                                        return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        assert await time() == pytest.approx(1.6) and slow.status == 'executing'
        await slow.cancel()
        assert slow.status == 'canceled' and await time() == pytest.approx(1.6)
        blocked = right(x=1.5, y=-.5)  # the wall stops it
        with pytest.raises(RPCError, match='right.move was aborted: stalled'):
            await blocked
        assert (blocked.status, blocked.result) == ('aborted', {'x': pytest.approx(1.2)})
        with pytest.raises(RPCError, match='a goal needs a tip pose'):
            await left({})  # a goal the backend cannot take ends as it starts
    run(Graph(tmp_path, backend='pair_pose_toy'), {**world(), **arms()}, program)


def test_a_goal_ended_after_its_program_was_cancelled_warns_of_no_other_server(tmp_path, capfd):
    """The result of a goal a cancelled program left still comes, once the goal is stopped: its client takes it,
    instead of warning that another action server may be answering."""
    from tests.ros.support import Graph

    async def program(io):
        goal = io.device('left').action('move')(x=1.1, y=-.5)
        await asyncio.wait([goal.accepted])
        goal.task.cancel()  # as a cancelled program's tasks are
        await asyncio.gather(goal.task, return_exceptions=True)
        await io._stop(['left'])  # as its scope then stops what it commanded
        await asyncio.sleep(1.)
    run(Graph(tmp_path, backend='pair_pose_toy'), {**world(), **arms()}, program)
    assert 'unexpected result response' not in capfd.readouterr().err


def test_loops_at_different_rates_share_a_stepped_world(tmp_path):
    """Each loop ticks on its own deadlines, the world advances once for all of them and rests while a tick's body
    runs, and with nothing waiting on it."""
    from tests.ros.support import Graph

    async def program(io):
        world, target, ticks = io.world, io.device('finger').topic('target'), {10: [], 5: []}

        async def loop(hz, count):
            async for now in world.rate(hz):
                ticks[hz].append(now)
                await target.publish({'data': .1})  # a command in flight: the world rests
                await asyncio.sleep(.02)  # and a slow body costs no world time
                if len(ticks[hz]) == count:
                    break
        await asyncio.gather(loop(10, 7), loop(5, 4))
        assert ticks[10] == pytest.approx([.1 * tick for tick in range(7)])
        assert ticks[5] == pytest.approx([.2 * tick for tick in range(4)])
        await asyncio.sleep(.3)
        assert (await world.now()).time_s == pytest.approx(.6)  # nothing waits: the world is paused
        assert await world.wait(.4) == pytest.approx(1.) and (await world.now()).time_s == pytest.approx(1.)
    run(Graph(tmp_path), {**world(), **finger()}, program)


def test_an_abandoned_wait_stops_a_stepped_world(tmp_path):
    from tests.ros.support import Graph

    async def program(io):
        world = io.world
        assert await world.wait(.1) == pytest.approx(.1)
        with pytest.raises(TimeoutError):
            await asyncio.wait_for(world.wait(1e6), .3)
        await asyncio.sleep(.5)  # the driver stops the world's wait
        reached = (await world.now()).time_s
        assert reached > .1
        await asyncio.sleep(.3)
        assert (await world.now()).time_s == reached  # it advanced no further
        # A refused command ends a wait, which raises for every task waiting on the world.
        await io.device('finger').topic('target').publish({'data': float('nan')})
        await asyncio.sleep(.2)
        with pytest.raises(OperationError, match='ended early by a refused command'):
            await asyncio.gather(world.wait(.5), world.wait(1.))
    run(Graph(tmp_path), {**world(), **finger()}, program)


def test_where_time_passes_by_itself_the_same_waits_only_block(tmp_path):
    """A realtime simulator, as the real world is: the world's operations are the same, and it never pauses."""
    from tests.ros.support import Graph

    async def program(io):
        world, left = io.world, io.device('left').action('move')
        start = await world.now()
        await asyncio.sleep(.3)
        before = await world.now()
        assert before.time_s > start.time_s and before[1:] == (None, None)  # it runs unwatched, without a budget
        assert await world.wait(.3) >= before.time_s + .3 - 1e-9
        assert await left(x=.7, y=-.5) == {'x': pytest.approx(.7)}  # 0.2 s away
        ticks = []
        async for now in world.rate(5):
            ticks.append(now)
            if len(ticks) == 4:
                break
        assert [later - earlier for earlier, later in zip(ticks, ticks[1:])] == pytest.approx([.2] * 3, abs=.11)
    run(Graph(tmp_path, mode='realtime', backend='pair_pose_toy'), {**world(), **arms()}, program)
