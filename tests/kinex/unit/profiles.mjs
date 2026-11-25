// Read exported workspaces with Kinex's own profile schema:
//   node tests/kinex/unit/profiles.mjs KINEX_CORE_DIST WORKSPACE...
// Prints one {robot, devices} JSON object per workspace; exits nonzero when a profile fails to parse.
import path from 'node:path'
import { pathToFileURL } from 'node:url'

const [dist, ...workspaces] = process.argv.slice(2).map(value => path.resolve(value))
const { ProfileStore } = await import(pathToFileURL(path.join(dist, 'device/profile/store.js')).href)
const results = []
for (const directory of workspaces) {
  const robot = await ProfileStore.readManifest(directory)
  const devices = await ProfileStore.listDevices(directory)
  results.push({
    robot: robot?.identity?.id ?? null,
    devices: Object.fromEntries(devices.map(d => [d.name, {type: d.type, operations: Object.keys(d.operations)}])),
  })
}
console.log(JSON.stringify(results, null, 2))
