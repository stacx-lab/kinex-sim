// Native cross-repository smoke: start kinex-sim, export a workspace, prepare its
// Kinex Python, then: node tests/kinex/ros/workspace.mjs WORKSPACE.
import assert from 'node:assert/strict'
import { access, mkdtemp, rm } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import path from 'node:path'
import { setTimeout as delay } from 'node:timers/promises'
const { createKinexRuntime } = await import(process.env.KINEX_CORE ?? '../../../../kinex/packages/core/dist/index.js')
const directory = path.resolve(process.argv[2])
const runtime = createKinexRuntime({directory, enableRecording:true})
const owned = []
const host = {sessionID:'native-simple',ask:async()=>{}}
const call = (name, args) => runtime.tools.call(name, args, host)
const latencies = []
async function exec(body, {error=false, timeout_s=120}={}) {
  const started = performance.now()
  const result = await call('exec', {python:`import asyncio
from kinex import device, finish, lib, runtime, seconds, world
from kinex.errors import OperationError
async def measured(name, operation, time_s):
    topic = device(name).topic(operation)
    async with asyncio.timeout(5):
        while True:
            state = await topic.next()
            if seconds(state.header.stamp) >= time_s - 1e-8:
                return state
async def clock():
    return (await world.now()).time_s
async def after(seconds):
    """World time once seconds of it have passed."""
    return await world.wait(seconds)
async def main():
${body.split('\n').map(line=>'    '+line).join('\n')}
asyncio.run(main())`,timeout_s})
  latencies.push(performance.now() - started)
  assert.equal(Boolean(result.isError),error,result.output)
  if (error) return result.output
  console.log(result.output)
  return JSON.parse(result.output).result
}
try {
  const profiles = await runtime.devices.list()
  assert.deepEqual(Object.keys(profiles.find(d=>d.name==='world').operations), ['now','wait','stop'])
  assert.deepEqual(Object.keys(profiles.find(d=>d.name==='gripper').operations), ['joint_state','target','stop'])
  for (const name of ['world','arm','gripper','camera_agentview','camera_wrist']) {
    await (await runtime.devices.load(name)).ready
    owned.push(name)
    console.log('loaded',name)
  }
  const started = await runtime.recording.start({specs:{gripper:{rate:20},camera_agentview:{rate:10}},sessionID:host.sessionID})
  assert.deepEqual(started.refusals,{})
  const first = await exec(`assert runtime.describe('gripper.target')['message'] == {'data': 'double'}
from kinex.communication import scope
assert set(runtime.describe('world.wait')['request']) == {'seconds'}
assert all(not name.endswith(('/task/status','/task/instruction')) for name,_ in scope().bridge.node.get_topic_names_and_types())
state = await device('arm').topic('joint_state').next(timeout_s=5)
await lib.frames.lookup_transform('world:base','gripper:tcp',state.header.stamp)
image, depth = await lib.images.camera_frame('camera_agentview')
assert depth is None and image.header.frame_id == 'camera_agentview/base'
finish({'time':await clock(),'stamp':{'sec':state.header.stamp.sec,'nanosec':state.header.stamp.nanosec},'camera':[image.width,image.height]})`)
  await exec(`await device('gripper').topic('target').publish({'data':0.})
finish({'sent':True})`)
  const closed = await exec(`from builtin_interfaces.msg import Time
now=await after(1.5)
state=await measured('gripper','joint_state',now)
# Historical lookup survives the command's separate exec turn.
await lib.frames.lookup_transform('world:base','gripper:tcp',Time(**${JSON.stringify(first.stamp)}))
finish({'finger':state.position[0]})`)
  assert.ok(closed.finger < .01)
  await exec(`gripper=device('gripper')
await gripper.service('stop')()
state=await measured('gripper','joint_state',await after(.3))
assert state.position[0] < .01
await gripper.topic('target').publish({'data':.04})
state=await measured('gripper','joint_state',await after(1.5))
assert state.position[0] > .03
finish({'open':state.position[0]})`)
  // A failed script stops what it commanded: the gripper holds open instead of closing.
  await exec(`await device('gripper').topic('target').publish({'data':0.})
raise RuntimeError('intentional failure cleanup')`,{error:true})
  await exec(`state=await measured('gripper','joint_state',await after(.3))
assert state.position[0] > .03
finish({'failure_cleanup_held':True})`)
  // Loops at different rates share the world: each ticks on its own deadlines, and the world advances once for both.
  const ticks = await exec(`start, ticks = await clock(), {10: [], 4: []}
async def loop(hz, count):
    async for now in world.rate(hz):
        ticks[hz].append(round(now - start, 6))
        if len(ticks[hz]) == count:
            break
await asyncio.gather(loop(10, 6), loop(4, 3))
finish({'ticks': ticks, 'elapsed': round(await clock() - start, 6), 'budget': (await world.now())[1:]})`)
  assert.deepEqual(ticks.ticks, {10: [0, .1, .2, .3, .4, .5], 4: [0, .25, .5]})
  assert.equal(ticks.elapsed, .5)
  // A program's waits are one wait on the world at a time; an abandoned wait stops the world.
  await exec(`start=await clock()
assert [round(now - start, 6) for now in await asyncio.gather(world.wait(.2), world.wait(.1))] == [.2, .1]
pending=asyncio.create_task(world.wait(60))
await asyncio.sleep(.5)
pending.cancel()
await asyncio.gather(pending,return_exceptions=True)
await asyncio.sleep(.5)
before=await clock()
await asyncio.sleep(1)
assert await clock()==before
finish({'abandoned_wait_stopped':True})`)
  // A script killed mid-wait leaves no wait advancing the world.
  await exec(`await world.wait(60)`,{error:true,timeout_s:3})
  const settled = await exec(`before=await clock()
await asyncio.sleep(1)
later=await clock()
await world.wait(0)
finish({'advanced_s':later-before})`)
  assert.equal(settled.advanced_s,0)
  // A device stop cancels the running script that commanded it.
  const markers = await mkdtemp(path.join(tmpdir(),'kinex-smoke-'))
  const marker = path.join(markers,'commanded')
  const running = exec(`await device('gripper').topic('target').publish({'data':.04})
open(${JSON.stringify(marker)},'w').close()
await world.wait(60)`,{error:true})
  while (!await access(marker).then(()=>true,()=>false)) await delay(20)
  assert.ok(!(await call('device_stop',{deviceID:'gripper'})).isError)
  assert.match(await running,/device stop cancelled this invocation/)
  await rm(markers,{recursive:true,force:true})
  // Direct reads reuse the device's loaded connection, and a program attaches to a runtime that is already up.
  const reads = []
  for (let index = 0; index < 5; index++) {
    const started = performance.now()
    assert.ok(!(await call('device',{deviceID:'arm',action:'joint_state'})).isError)
    reads.push(performance.now() - started)
  }
  console.log('device read ms',reads.map(value=>value.toFixed(1)).join(' '))
  console.log('exec ms',latencies.map(value=>value.toFixed(0)).join(' '))
  const recording = await runtime.recording.stop()
  console.log('recording',JSON.stringify(recording))
  await runtime.devices.unload('gripper')
  await exec(`try:
    await lib.frames.lookup_transform('world:base','gripper:tcp',timeout_s=.1)
except Exception:
    finish({'unloaded_frame_unavailable':True})
else:
    raise AssertionError('unloaded frame remained available')`)
  await (await runtime.devices.load('gripper')).ready
  await exec(`await lib.frames.lookup_transform('world:base','gripper:tcp')
finish({'reloaded_frame_available':True})`)
} finally {
  for (const name of owned.reverse()) {
    try {await runtime.devices.unload(name)} catch(error) {console.error('unload',name,error);process.exitCode=1}
  }
  await runtime.dispose()
}
