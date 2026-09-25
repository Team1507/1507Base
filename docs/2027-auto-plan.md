# 2027 Autonomous Plan

**Status:** planning. Nothing here is implemented yet.
**Branch:** `SystemCore` (WPILib 2027, Commands v3)

This plan redesigns how auto routines are written, so that:
- routines read top to bottom, one line per thing that happens, with no `seq ->` lambdas;
- the robot can **do things while driving** (deploy the intake as it passes a point) without parallel groups;
- a routine can be driven by the **classic driver** (straight lines between points) or the **policy driver** (the trained neural network from Swerve-Policy-Playground);
- one routine can run on **either side of the field** (mirroring), with separate Left/Right routines still possible when a side needs its own tuning.

---

## 1. What 2026 taught us

Rebuilt2026 had 17 routine files (4 of them empty Red copies) for about 6 kinds of auto:

| Kind | 2026 routines | What it does |
|---|---|---|
| Preload only | Autoah Raymond | Shoot until 4 s |
| Human player | Human Player, Human Player Quest | Drive to outpost, deploy intake, creep into the wall, wait, return while retracting, shoot |
| Single sweep | Subway 6 Inch L/R, 18 Inch L/R, Footlong R | Cross bump, sweep the neutral zone with the intake down, retract on the way back, shoot |
| Double sweep | Double Subway (+Left), Across Middle | Two sweeps; first shot cut off by match time |
| Around the hub | Around the Hub | Sweep right, shoot, go around the hub, sweep left, shoot |
| Rush | Rush to Subway | Go to the far side first |

Lessons:
- **Every `seq ->` was "do X while driving to Y"** (deploy or retract the intake). One even had a single-branch `parallel`, which does nothing. The lambdas were confusing and only ever did one job.
- **Left/Right copies drifted apart.** Different nodes, different pass radii (0.2 vs 0.5), a separate "Quest" copy that differed in two lines. Without AprilTags the robot had no absolute position; each side was lined up by hand and tuned separately.
- **The robot reached XY and ignored heading.** `driveTo` finishes as soon as the position is within 5 cm, even if the robot is still turning, and nodes store typed angles instead of "face the hub". `pointToTarget` was added after every drive as a workaround.
- **Speed** was set per auto (0.5–1.0 × max in the chooser) and per leg (`withSpeed(MaxSpeed * 0.5)`).
- **Match-time cutoffs** (`shootUntil(13.5)`) were essential for multi-cycle autos.

---

## 2. Decisions made

| Decision | Choice |
|---|---|
| Driver chosen how | `new AutoSequence()` is the classic driver (the default). `new AutoSequence(Driver.POLICY)` switches to the policy |
| Policy and classic autos | **Separate routines** ("Sweep" and "Sweep – Policy"). They use different nodes anyway: classic needs extra nodes to steer around obstacles, the policy avoids them itself |
| Backup plan | Pick the classic version of the auto on the Driver Station |
| `driveTo` / `moveThrough` in autos | Removed; replaced by `endpoint` / `checkpoint` |
| Mirroring | Yes: normally 1–3 routines, mirrored to the side the robot starts on. Side-specific routines are still allowed |
| Speed modifiers with the policy | Kelly will add speed to the policy's training once this structure is final |
| Heading in the policy | Kelly will add a heading observation to training later. Until then our code turns the robot while the policy drives |
| Heading | The robot always turns toward the **node's heading** while driving. An endpoint finishes on **XY only** unless `.heading()` or `.facing(...)` is added, which also make it wait for the heading |
| Checkpoint accuracy | Named presets (`Accuracy.PRECISE / TIGHT / NORMAL / LOOSE`), each setting distance **and** heading tolerance, defined once in Constants so tuning a preset updates every auto. `NORMAL` when left out; exact numbers for rare cases |
| Heading on checkpoints | Comes from the node (store the 2026 bump node at 45° and the robot crosses at 45°). `.heading(deg)` / `.facing(...)` only override it; a checkpoint never waits for heading |
| Unknown side / position at enable | **Never stop the auto.** `resetPose` gets the pose close; QuestNav corrects it when it's tracking. The only give-up stays the existing stall detection |

---

## 3. How routines will read

### Path steps

| Step | Meaning | Does the sequence wait? | Robot |
|---|---|---|---|
| `.checkpoint(node)` | Drive **through** this node | Waits until the robot reaches it | **Keeps moving** |
| `.waypoint(node)` | A shaping point: keeps the route where you want it | No; it's not a step, only part of the route | Keeps moving |
| `.endpoint(node)` | Drive to this node and **stop**. Finishes on position, unless `.heading()` / `.facing(...)` is added | Waits until the robot has stopped there | Stops |

**The one rule to teach:** after a checkpoint, the robot keeps driving toward the next endpoint while the steps after the checkpoint run. To stop and then do something, use an endpoint.

How it works: when the sequence reaches a path step, the robot starts driving the whole route up to the next endpoint **in the background**. Path steps then only wait for "reached". Every other step (`intakeDeploy()`, `shoot()`) runs while the robot drives. If an action takes long enough that the robot passes the next checkpoint first, that checkpoint's wait finishes immediately; nothing is missed.

### Heading: `.heading()` and `.facing()`

Every node has a heading (`Node.at(x, y, degrees)`). While driving to a node, the robot **always turns toward that node's heading**. Whether an endpoint **waits** for the heading is up to the routine:

With `NODE_A = Node.at(1.2, 1.5, 45)`:

| You write | Robot turns toward | Endpoint waits for heading? |
|---|---|---|
| `.endpoint(NODE_A)` | 45° (the node's heading) | No: finishes on XY. Fastest |
| `.endpoint(NODE_A).heading()` | 45° | **Yes** |
| `.endpoint(NODE_A).heading(90)` | 90° (overrides the node) | **Yes** |
| `.endpoint(NODE_B).facing(Nodes.Hub.CENTER)` | The angle that points at the hub **from NODE_B's position**, e.g. 60° instead of NODE_B's 90° | **Yes** |

- **`.heading()`** is about the robot's own pose: wait for the node's heading, or `.heading(deg)` to override it.
- **`.facing(location)`** points the robot at something. The angle is computed once, from the node's position (where the robot will be), not re-aimed every loop, so it's steady while driving.
- **Waiting for heading is what fixes 2026's problem.** A plain `.endpoint(node)` behaves like 2026's `driveTo`: it turns on the way but finishes as soon as XY is reached, possibly mid-turn. When heading matters (shooting), add `.heading()` or `.facing()`. When it doesn't, leave them off and save the turning time.
- **On checkpoints**, heading counts, but loosely: see Checkpoint accuracy below. `.heading(deg)` and `.facing(...)` change which heading that is. Low priority; the node's own heading covers most cases (e.g. the bump node stored at 45°).
- **Waypoints** use their node heading too.
- **Mirroring and Red flipping are applied first,** so `.facing()` computes from the flipped/mirrored positions.
- Works with both drivers: they move the robot, and our heading controller turns it.

### Checkpoint accuracy

How close a checkpoint must be before it counts as reached, as a named preset. Each preset sets **distance and heading together**, so students never reason about degrees for a checkpoint. The values live in one place (Constants): change a preset once and every auto that uses it updates, the same idea as the `MotorConfig` presets. Typing `Accuracy.` in VS Code lists them, so the names don't have to be memorized.

| Preset | Distance | Heading | Use |
|---|---|---|---|
| `Accuracy.PRECISE` | 0.1 m | 5° | Lining up before a bump or a narrow gap |
| `Accuracy.TIGHT` | 0.2 m | 15° | Most 2026 checkpoints |
| `Accuracy.NORMAL` | 0.4 m | 30° | The default, when no preset is given |
| `Accuracy.LOOSE` | 0.7 m | 45° | Roughly there; fastest |

```java
.checkpoint(Nodes.SUBWAY_ENTRY)                          // NORMAL
.checkpoint(Nodes.OVER_BUMP, Accuracy.PRECISE)           // square up before the bump
.checkpoint(Nodes.MIDFIELD, 0.3, 20)                     // exact: 0.3 m and 20°, for rare cases
```

A checkpoint is reached when the robot is within **both** its distance and its heading tolerance. If the heading is behind (the bump needs 45° but the robot is at 70°), the robot slows at the checkpoint until the heading catches up, then continues. With `LOOSE` that almost never happens; with `PRECISE` it guarantees the angle.

Endpoints keep their own tighter tolerances (5 cm, and 3° when `.heading()` / `.facing()` is used), also in Constants.

The starting values come from the pass radii 2026 used (0.1–0.7 m) and will be tuned on the robot.

### Actions

| Step | Meaning |
|---|---|
| `.intakeDeploy()`, `.shootUntil(13.5)` | Subsystem steps (a one-line wrapper each, as today). The sequence waits for them to finish; the robot keeps driving if a route is active |
| `.intakeRollersOn()` and similar | For things that must **keep running** (rollers). The wrapper starts the command in the background and the list moves on immediately; a later step for that subsystem (`.intakeRollersOff()`) or the end of the auto stops it |
| `.waitSeconds(t)`, `.waitUntilTime(t)`, `.waitUntil(cond)` | As today |

**Why keep-running steps need special handling:** in 2026, `intakeHigh()` set the roller speed and finished at once; the roller subsystem remembered its speed. In 1507Base, when a command finishes, the subsystem's default command (`idle()`) takes over and stops the motor. So "rollers on" must be a command that keeps running, and in a step list a step that never finishes blocks everything after it. The wrapper handles this with a background start (`runInBackground(cmd)`, used only inside wrappers). **Students writing routines never see it**: to them it's a normal step.

`parallel` / `race` / `deadline` stay available for rare advanced cases, but no 2026 auto would need them.

### Speed

```java
new AutoSequence().maxSpeed(0.8)                 // whole auto (was the chooser's MAX_SPEED * 0.8)
.slow().checkpoint(Nodes.SUBWAY_ENTRY)           // the leg into this node only
```

### Example: 2026 single sweep, new style (classic)

2026 `AutoSubway6inchRight`, 20 lines with two `parallel(seq -> ...)` groups:

```java
@Autonomous(name = "Sweep")
public final class SweepAuto extends AutoOpMode {
    @Override
    protected Command build() {
        return new AutoSequence()                  // classic driver (the default)
            .maxSpeed(0.8)
            .resetPose(Nodes.Start.RIGHT)
            .checkpoint(Nodes.OVER_BUMP, Accuracy.TIGHT)
            .intakeDeploy()                        // deploys while driving on
            .slow().checkpoint(Nodes.SUBWAY_ENTRY, Accuracy.PRECISE)
            .slow().checkpoint(Nodes.SUBWAY_EXIT, Accuracy.TIGHT)
            .intakeRetract()                       // retracts while driving on
            .checkpoint(Nodes.BEFORE_BUMP, Accuracy.TIGHT)
            .checkpoint(Nodes.OVER_BUMP, Accuracy.TIGHT)
            .endpoint(Nodes.Start.RIGHT).facing(Nodes.Hub.CENTER)
            .shootUntil(19.99)
            .build();
    }
}
```

### Example: the same auto with the policy

Fewer nodes: the policy steers around the bump edges and hub itself.

```java
@Autonomous(name = "Sweep - Policy")
public final class SweepPolicyAuto extends AutoOpMode {
    @Override
    protected Command build() {
        return new AutoSequence(Driver.POLICY)
            .resetPose(Nodes.Start.RIGHT)
            .checkpoint(Nodes.SUBWAY_ENTRY)
            .intakeDeploy()
            .checkpoint(Nodes.SUBWAY_EXIT)
            .intakeRetract()
            .endpoint(Nodes.Start.RIGHT).facing(Nodes.Hub.CENTER)
            .shootUntil(19.99)
            .build();
    }
}
```

### Example: double sweep with a time cutoff

```java
            .endpoint(Nodes.SHOOT_SPOT).facing(Nodes.Hub.CENTER)
            .shootUntil(13.5)                      // first cycle must end by 13.5 s
            .checkpoint(Nodes.OVER_BUMP)           // second cycle
            ...
```

### Example: human player

```java
            .endpoint(Nodes.Outpost.APPROACH)
            .intakeDeploy()
            .creep().driveForwardMeters(0.15)      // an endpoint relative to where the robot is
            .waitSeconds(1.5)
            .checkpoint(Nodes.Outpost.APPROACH)
            .intakeRetract()
            .endpoint(Nodes.Start.RIGHT).facing(Nodes.Hub.CENTER)
            .shootUntil(19.5)
```

---

## 4. Mirroring

Routines are written for **one side** (the right side, from the Blue driver station). At enable, the auto decides which side the robot is on and mirrors every node, heading and facing target across the field's long centerline if needed. This works together with Red flipping, which already exists.

**How the side is chosen:**

| Situation | Side comes from |
|---|---|
| QuestNav tracking (AprilTags) | The robot's measured position at enable |
| No QuestNav | The robot's pose from the dashboard seed buttons (Seed Left / Center / Right, as in 2026), or the routine's `resetPose` node |
| Routine says `.side(Side.LEFT)` or `.noMirror()` | Exactly what it says. For side-specific routines |

**The auto never stops itself** over position or side. The side is decided once, at enable, from the best position available (QuestNav, else the seed buttons). If nothing tells it the side, it runs the route as written. `resetPose` then sets the pose close to the start node, and QuestNav corrects it once it's tracking. The only give-up remains the existing stall detection (robot pushing against something).

**Advanced goal (later, policy only):** "run to the center, come back and shoot" from **anywhere** on the field. QuestNav gives the starting position, and the policy drives from there to the first node without hitting anything. The classic driver can't do this safely, because a straight line from an arbitrary spot may cross the hub.

Like Red flipping, the mirror line depends on the season's field (for 2026 it was the long centerline, since the hub is centered). It is set once per season next to `FieldFlip`.

---

## 5. The two drivers

Both implement the same interface: drive a route (list of nodes, each a checkpoint, waypoint or endpoint), report each node as it's reached, and give up safely if stuck. The give-up is logged and releases every remaining wait, so the auto continues instead of hanging.

### Classic driver

Built from today's proven code:
- checkpoints and waypoints: `moveThroughPose` logic (constant speed, pass radius);
- endpoints: `driveToTarget` logic. Finishes on XY by default; with `.heading()` / `.facing()`, only when heading is also within tolerance. The existing stall give-up stays;
- heading: the existing heading controller, always turning toward the current node's heading (or its `.heading` / `.facing` override).

### Policy driver

- Runs the trained network **on SystemCore, in Java**. The network is small (8 inputs → 256 → 256 → 3 outputs; the third, rotation, is unused), 68,352 multiply-adds per loop, well under 1 ms. No extra libraries.
- Weights are exported from Stable-Baselines3 to a file in the robot's deploy folder.
- Each loop it builds the same 8 inputs the training used (velocity, position, vectors to the current and next node), runs the network, and drives with the output. Our heading controller adds the turning.
- **Endpoints:** the policy gets the robot close, then the classic endpoint logic finishes the last stretch precisely, until the policy is trained for tight endpoints.
- A test checks that Java produces the same outputs as Python for a set of saved inputs.

---

## 6. Build-time checks (like `NodeBoundsTest`)

- **Policy routes:** consecutive route nodes are at most 5 m apart.
- **Classic routes:** no straight segment between consecutive nodes crosses a field element (uses the corner polygons already in `Nodes.FieldElements`).
- **All routes:** no node inside a field element or outside the field (already checked for nodes today).

These run on every build, so a bad route fails before it reaches the robot.

---

## 7. What the policy training needs from this

For Swerve-Policy-Playground, once this structure is final:

| Training change | Why |
|---|---|
| Separate pass radius for endpoints (tight) and checkpoints (0.65 m) | Endpoints must be accurate |
| Checkpoint accuracy (distance and heading tolerance) as an input | A `PRECISE` 0.1 m checkpoint asks for more than the 0.65 m it was trained on; without this it may overshoot and come back |
| Speed limit as an input | So `maxSpeed` / `slow()` work with the policy |
| Heading input | Later; our heading controller covers it until then |
| Robot's real top speed and acceleration limits (15 / 10 m/s², see `SwerveConfig`) | The current simulator lets the robot accelerate almost instantly. On the real robot, with acceleration limits, the policy could overshoot |
| Waypoint input (a node that shapes the route but isn't a stop) | Already covered: the policy sees current and next node |
| Each season: the new field's obstacles | Avoidance is learned from the field layout |
| Export script: SAC actor weights → file for the robot | For the Java policy driver |

---

## 8. Open questions

None right now. Answered so far:
- Default driver: classic; `Driver.POLICY` to switch.
- Heading: always turn toward the node's heading; endpoints wait for it only with `.heading()` / `.facing()`.
- Unknown side: never stop; run as written, `resetPose` + QuestNav correct the pose.
- Keep-running steps: handled inside wrappers; no new step for students.
- Checkpoint accuracy: named presets (`PRECISE / TIGHT / NORMAL / LOOSE`) in Constants, `NORMAL` by default, exact numbers for rare cases.
- Heading on checkpoints: from the node; `.heading(deg)` / `.facing()` override it, never wait.

---

## 9. Feasibility review (2026-09-25)

Checked against WPILib 2027 alpha 7, Commands v3, 1507Base (`SystemCore` branch), Rebuilt2026 / Rebuilt2026_V2 and Swerve-Policy-Playground, with throwaway tests where behavior mattered.

### Verified to work

| Plan item | How it was checked | Result |
|---|---|---|
| Route drives in the background while other steps run | v3 test: auto forks a "route" (swerve), then runs intake steps | ✅ route kept driving during the intake steps |
| Checkpoint "wait until reached" | Same test | ✅ |
| Keep-running wrappers (rollers), stopped by a later step for the same subsystem | Same test | ✅ the later intake step interrupted the rollers |
| Keep-running actions stop when the auto ends | Same test | ✅ |
| Disable cancels the route and all background actions | Same test (root canceled) | ✅ |
| Swerve's `idle()` resumes between route segments | Same test | ✅ |
| Next step starts in the same loop the previous one ends | v3 test | ✅ no stutter between steps |
| Autos can receive the `Robot` (no per-subsystem `AutoBuilder` registration) | `OpModeRobot` source: tries a Robot constructor, then no-argument | ✅ |
| Every auto built and route-checked in a unit test | Test constructed `Robot` (about 7 s, once) and built `DriveForwardAuto` | ✅ |
| Policy runs in plain Java math | Python: 2 layers + ReLU + output + tanh vs Stable-Baselines3 `predict`, 1,000 random inputs | ✅ max difference 0.000001 |
| Default heading behavior matches 2026 | 2026 `driveTo` and `moveThrough` both turned toward the node heading | ✅ ported autos behave the same by default |
| 2026 robot's CANivore ("CAN-EATER") on SystemCore | WPILib SystemCore testing notes for Phoenix 6 | ✅ supported (needs the CANivore USB package on SystemCore) |

### Required implementation details (found by the tests)

1. **The route must be started by the auto's top-level command.** In v3, a background (forked) command runs only as long as the command that started it. If a checkpoint step started the route and then finished, the route would be killed. So `AutoSequence.build()` produces one top-level command that starts routes and keep-running actions itself, instead of `Command.sequence(...)`. Wrappers **register** keep-running actions with it rather than starting them.
2. **No swerve steps between a checkpoint and its endpoint.** A step that uses swerve (`resetPose`, `stop`, a turn) while the route drives **cancels the route** (verified). `build()` must reject this with a clear message. The heading options (`.heading()`, `.facing()`) replace the 2026 pattern of driving and then turning.
3. **The auto waits for the route at the end.** If the step list ends while a route is still driving, the route is canceled (verified). `build()` adds an automatic "wait for the route to finish" at the end.
4. **A canceled or stuck route releases every waiting checkpoint** and logs why, so the auto continues instead of hanging.
5. **Warm-up:** the first build of an auto took 34 ms, later builds 0.8 ms (Java loading classes). Building only at enable would freeze the robot for that long at the start of auto (longer on SystemCore). Build once when the auto is **selected** as a warm-up, then again at enable.

### Gaps in the plan (to add)

| Gap | Where it showed up | Proposed |
|---|---|---|
| **Time cutoffs on driving** | V2's `moveThroughBy(node, 0.2, 1.6)` / `driveToBy(node, 5.9)` | `.checkpoint(node).by(1.6)`: if not reached by 1.6 s of auto time, count it as reached and move on (logged) |
| **Side without QuestNav** | Mirroring needs to know the side; 1507Base has no seed buttons (they were in the 2026 dashboard) | Add Seed Left / Center / Right dashboard buttons that set the pose |
| **`.facing(Hub.CENTER)` needs field locations** | `Nodes.FieldElements.Hub` has corners but no `CENTER` | Add `CENTER` (and similar) locations |
| **Start driving from a stop while doing something** | Not needed by any 2026 auto (every `parallel` followed a moving step) | Optional later: a `.depart()` step ("start driving now; the next steps happen on the way") |
| **CANivore not in the health log** | `RobotHealthLog` logs SystemCore's ports only; the 2026 swerve is on a CANivore | Log the CANivore's bus utilization too |

### What running 2026 autos on the 2026 robot needs (outside this plan)

The auto framework is only part of it. To put SystemCore on last year's robot and run its autos:

| Needed | Size | Notes |
|---|---|---|
| **Port the 6 mechanisms** from Rebuilt2026_V2 (Agitator, Feeder, Hopper, IntakeArm, IntakeRoller, Shooter) | About 1,500 lines | 2027 package names, Commands v3, the new `Motor1507` goal methods (degrees/RPM), `MotorConfig` presets, `motor(...)`. V2 already uses `Subsystem1507`, so the structure carries over |
| **Port the 2026 nodes** (Start, Midfield, Outpost, Hub) | Data only | From Rebuilt2026's `Nodes` |
| **Rewrite the auto wrappers** (`intakeDeploy`, `intakeHigh`, `shootUntil`, `pointToShoot`, ...) | Small | `intakeHigh()` / `intakeLow()` were "set speed and finish"; with `idle()` default commands they become keep-running wrappers |
| **Port the 2026 shooter logic** (model-based shooting, `pointToShoot`) | Medium | Season code |
| **Swerve for the 2026 modules** | Paste | Tuner X paste zone: drive 6.12, steer 21.43, coupling 3.57, wheel 1.952 in, Pigeon ID 30, bus "CAN-EATER" (`Constants.CAN_BUS`) |
| **QuestNav** | Waiting | Parked until its 2027 build. Until then position comes from `resetPose` and odometry, as in 2026 |

This belongs in a **2026-robot test project** (a copy of 1507Base), not in 1507Base itself.

---

## 10. Build order

0. **Plan fixes above:** time cutoffs (`.by`), seed buttons, `Hub.CENTER`, CANivore logging.
1. **Endpoint heading** in the classic logic: finish on XY by default; with `.heading()` / `.facing()`, finish on position AND heading. Small, and it fixes the 2026 problem on its own.
2. **Route steps in `AutoSequence`:** `Driver` (classic default), `checkpoint` (with `Accuracy` presets in Constants), `waypoint`, `endpoint`, `heading` / `facing` (endpoints first, checkpoint overrides later), `maxSpeed`; the top-level auto command that starts routes and keep-running actions (section 9); the no-swerve-steps-inside-a-route check; the automatic wait for the route at the end; the selection-time warm-up build; give-up handling; logging (`Auto/Route/...`). Remove `driveTo`/`moveThrough` steps.
3. **Mirroring:** side detection at enable, `.side()`, `.noMirror()`, the season mirror line.
4. **Build checks** (section 6), run by a test that constructs `Robot` and builds every auto; tests for each step type, mirroring and Red flipping.
5. **Port one 2026 auto of each kind** as the examples and the template. Update the Autonomous wiki page.
6. **1507Labs:** the policy driver (weights loader, Java network, parity test), then promote to 1507Base once it has driven a real robot.

Steps 1–5 need no robot and no trained policy. Step 6 needs the exported policy.
