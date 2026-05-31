# /new-game-field — Build a sim field + custom IronMaple arena from a game manual

Use this playbook when a **new FRC season** is announced and **IronMaple has not yet
shipped a `Arena<year><Name>` field map**. It walks you from a game-manual PDF to:

1. a **custom IronMaple `SimulatedArena`** (barriers/obstacles + game-piece registration),
2. a **curated `GamePieceSpawner`** (a sane number of starting pieces, not the full grid),
3. a **2D web field renderer** in `src/main/resources/web/index.html` (the `testSim`
   browser view), and
4. the wiring + verification to make `./gradlew simulateJava -PtestSim` show it.

This is the generalization of the 2026 Rebuilt work (`Arena2026Patch`, `GamePieceSpawner`,
`DemoIntake`, and the field drawing in `index.html`). When IronMaple later ships the real
arena, most of this becomes a thin patch (see **Step 8**).

> **Input:** the official game manual PDF (field drawings + dimensions), plus the season's
> **AprilTag layout JSON** from WPILib. Ask the user to attach the PDF, or point you at a
> copy you can `Read`. If you can only get a paper/− image of the field drawing, that is
> enough for geometry — extract coordinates from the dimensioned field layout pages.

---

## Step 0 — Orient yourself in the coordinate frame

Everything uses the **WPILib field coordinate frame**, the same one IronMaple and the web
renderer assume:

- Origin `(0, 0)` at the **bottom-left corner** as seen from the **Blue** driver station.
- **+X** points down-field toward the **Red** wall (length, ~16.5 m in recent games).
- **+Y** points to the **Blue** driver's **left** (width, ~8.2 m).
- Headings are CCW-positive radians, 0 = facing +X.
- **Z** is up; only projectiles and scoring targets use Z.

The game manual usually dimensions the field in **inches from a corner**. Convert with
`edu.wpi.first.math.util.Units.inchesToMeters(...)` and **never** hard-code raw doubles
without a comment showing the inch source (`// 53" trench bar`).

Field length/width come from the manual; cross-check against the AprilTag JSON's
`field` block (`length`, `width`) — they must agree.

---

## Step 1 — Extract a geometry inventory from the manual

Read the PDF and produce a **plain-text inventory** before writing any code. This is the
single most important step; everything downstream is mechanical once it exists.

Capture, for the whole field (you can mirror Blue→Red later, but record what the manual shows):

**A. Field envelope**
- `fieldLength` (X, metres), `fieldWidth` (Y, metres).

**B. Obstacles / barriers** — anything a robot collides with. For each:
- a label (e.g. `TRENCH`, `HUB`, `TOWER`, `REEF`),
- shape: rectangle (`w × h` in metres) or polygon,
- center pose `(x, y, headingRad)` in field metres,
- whether it is **symmetric** (one Blue + one Red mirror across the field center).

**C. Game pieces** — for each *type*:
- `type` name (the string IronMaple keys on, e.g. `"Fuel"`, `"Note"`, `"Coral"`),
- physical shape (disc radius, or box), **height** (Distance), **mass** (Mass),
- **starting positions** on the field (list of `(x, y)`), and which are near which wall.

**D. Scoring targets** — for each goal/hub:
- center `(x, y)` and **scoring Z height** (the height a piece must reach),
- a scoring **radius/tolerance**,
- which alliance owns it.

**E. Alliance zones** (if the game has them)
- the X (or boundary line) that divides own-zone from neutral/opponent.

**F. AprilTags** — do **not** transcribe these from the PDF. Load them from WPILib:
```java
edu.wpi.first.apriltag.AprilTagFieldLayout layout =
    edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
        edu.wpi.first.apriltag.AprilTagFields.kDefaultField); // tracks the season default
```
For the **web renderer** you'll need a static JS array of `{id, fx, fy, yaw}` — generate it
by reading the same JSON (`edu/wpi/first/apriltag/<season>.json` on the WPILib classpath),
not by hand.

Write this inventory into the PR description or a scratch comment. Have the user sanity-check
it against the manual before proceeding — a wrong dimension here propagates everywhere.

---

## Step 2 — Decide: does IronMaple already have this arena?

```bash
# List the season arenas shipped in the IronMaple build (inside the YAGSL jar on the classpath)
JAR=build/libs/*.jar   # or the yagsl/maplesim vendordep jar
unzip -l $JAR | grep -iE 'ironmaple/simulation/seasonspecific/.*Arena'
```

- **If an `Arena<year><Name>.class` exists** → you do **not** need a custom arena. Use it via
  `SimulatedArena.getInstance()`, verify its obstacles against your inventory, and write only
  an `Arena<year>Patch` (Step 7) for any discrepancies, plus the spawner (Step 6) and web view
  (Step 9). Skip Steps 3–5.
- **If it does not exist** → build a custom arena (Steps 3–5).

`SimulatedArena.getInstance()` lazily constructs the season's default arena and caches it in a
static `instance` field. To make the library use **your** arena instead, call
`SimulatedArena.overrideInstance(new MyArena())` **before the first `getInstance()` call**
(i.e. before `SwerveFactory.build()` runs). See Step 8 for the wiring point.

---

## Step 3 — Inspect the IronMaple API you'll extend (read-only)

These are the exact superclasses/records you build against (verified signatures — confirm with
`javap -p <class>` against the current jar, versions drift):

```
SimulatedArena (abstract)
  protected SimulatedArena(SimulatedArena.FieldMap fieldMap)
  static void overrideInstance(SimulatedArena)
  static SimulatedArena getInstance()
  void addGamePiece(GamePieceOnFieldSimulation)        // place a grounded piece
  void addGamePieceProjectile(GamePieceProjectile)     // launch an in-flight piece
  boolean removeGamePiece(GamePieceOnFieldSimulation)
  void clearGamePieces()
  Set<GamePieceOnFieldSimulation> gamePiecesOnField()  // synchronized — safe off-thread
  Set<GamePieceProjectile> gamePieceLaunched()
  void simulationPeriodic()                            // advances dyn4j 5×4ms sub-ticks
  // override in your subclass:
  void placeGamePiecesOnField()                        // called on reset / "Reset Field"

SimulatedArena.FieldMap (abstract — your obstacle list)
  protected void addRectangularObstacle(double widthM, double heightM, Pose2d center)
  protected void addBorderLine(Translation2d a, Translation2d b)
  protected void addCustomObstacle(org.dyn4j.geometry.Convex shape, Pose2d center)

GamePieceOnFieldSimulation.GamePieceInfo  (record — the piece spec)
  GamePieceInfo(String type, org.dyn4j.geometry.Convex shape,
                Distance gamePieceHeight, Mass gamePieceMass,
                double linearDamping, double angularDamping,
                double coefficientOfRestitution)

GamePieceOnFieldSimulation extends org.dyn4j.dynamics.Body implements GamePiece
  GamePieceOnFieldSimulation(GamePieceInfo info, Pose2d placement)
  Pose2d getPoseOnField();  Pose3d getPose3d();  String getType()

GamePieceProjectile implements GamePiece
  GamePieceProjectile(GamePieceInfo info, Translation2d initialPosition,
                      Translation2d robotVelocityMPS, ChassisSpeeds chassisSpeeds,
                      Rotation2d shooterFacing, Distance launchHeight,
                      LinearVelocity launchSpeed, Angle launchElevation)
  void launch()
  GamePieceProjectile withTargetPosition(Supplier<Translation3d>)
  GamePieceProjectile withTargetTolerance(Translation3d)
  GamePieceProjectile withHitTargetCallBack(Runnable)
  GamePieceProjectile enableBecomesGamePieceOnFieldAfterTouchGround()
  GamePieceProjectile withProjectileTrajectoryDisplayCallBack(Consumer<List<Pose3d>>)
```

**Decoding values not exposed by the public API** (e.g. a hub's scoring Z height): the 2026
hub poses lived in `protected static` fields. Recover them with `javap -p -c <class>` and read
the static initializer, or reflect them at runtime. Always leave a comment recording the source
(`// decoded from RebuiltHub static initialiser: Translation3d(x, y, 1.5748)`).

---

## Step 4 — Write the custom arena + field map

Create `src/main/java/org/frc5010/common/drive/swerve/arena/Arena<Year><Name>.java`.

```java
package org.frc5010.common.drive.swerve.arena;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/** Custom IronMaple arena for the <YEAR> <NAME> game, built from the game manual
 *  because IronMaple has not shipped Arena<Year><Name> yet. Replace with the library
 *  arena once it exists (see Arena<Year>Patch). All geometry in the WPILib field frame. */
public class Arena<Year><Name> extends SimulatedArena {

  public Arena<Year><Name>() {
    super(new FieldMap<Year><Name>());
  }

  /** Obstacle list — every collision body on the field. */
  static final class FieldMap<Year><Name> extends SimulatedArena.FieldMap {
    FieldMap<Year><Name>() {
      double in = 0.0254; // inches → metres, for readability below

      // ---- field perimeter walls (so robots/pieces don't escape) ----
      double L = Units.inchesToMeters(/* field length in */), W = Units.inchesToMeters(/* width */);
      addBorderLine(new Translation2d(0, 0), new Translation2d(L, 0));
      addBorderLine(new Translation2d(L, 0), new Translation2d(L, W));
      addBorderLine(new Translation2d(L, W), new Translation2d(0, W));
      addBorderLine(new Translation2d(0, W), new Translation2d(0, 0));

      // ---- obstacles from the Step-1 inventory (mirror Blue ↔ Red where symmetric) ----
      // addRectangularObstacle(widthM, heightM, centerPose)
      addRectangularObstacle(53 * in, 12 * in, new Pose2d(4.6251, 1.4315, Rotation2d.kZero)); // TRENCH blue-near
      // ... one call per obstacle ...

      // Non-rectangular obstacle (e.g. hexagon, triangle):
      // addCustomObstacle(org.dyn4j.geometry.Geometry.createPolygon(verts...), centerPose);
    }
  }

  /** Spawn the curated starting pieces. Called on construction and on "Reset Field".
   *  Delegate to GamePieceSpawner so the curated layout lives in one place. */
  @Override
  public void placeGamePiecesOnField() {
    GamePieceSpawner.spawnInitialPieces(this);
  }
}
```

> **Friction/restitution for obstacles:** `addRectangularObstacle` applies IronMaple's
> standard obstacle material. If you ever build a body by hand (the `Arena*Patch` reflective
> path), match the library's `friction = 0.6`, `restitution = 0.3` (see `Arena2026Patch`).

---

## Step 5 — Define each game-piece type

Create one small class per piece type (mirrors `RebuiltFuelOnField` / `RebuiltFuelOnFly`).
Put the `GamePieceInfo` spec in a shared constants class so the grounded and projectile
variants share it.

```java
// GamePieces<Year>.java — shared specs
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;
import org.dyn4j.geometry.Geometry;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;

public final class GamePieces<Year> {
  private GamePieces<Year>() {}
  // type, shape, height, mass, linearDamping, angularDamping, restitution
  public static final GamePieceInfo FUEL = new GamePieceInfo(
      "Fuel", Geometry.createCircle(Units.inchesToMeters(2.5)),
      Inches.of(5), Pounds.of(0.3), 0.3, 0.3, 0.4);
}

// FuelOnField.java
public class FuelOnField extends GamePieceOnFieldSimulation {
  public FuelOnField(Translation2d pos) {
    super(GamePieces<Year>.FUEL, new Pose2d(pos, Rotation2d.kZero));
  }
}
```

For a **launched** piece, construct a `GamePieceProjectile` directly with `GamePieces<Year>.FUEL`
and configure the target/scoring callback — see `DemoIntake.fireFuel()` for the live pattern.

**Ballistic aiming (reuse this).** Given a fixed elevation angle θ, the launch speed to reach a
target at horizontal distance `d` and height delta `dz` is:
```
v = sqrt( g · d² / ( 2 · cos²θ · ( d·tanθ − dz ) ) )      g = 9.80665
```
Use a steep angle (~65°) for elevated goals, a flatter one (~40°) for ground lobs, clamp the
result to a sane `[2, 20] m/s`, and return the min when the denominator ≤ 0 (target too close).
This is exactly what `DemoIntake.computeLaunchSpeed(...)` implements — copy it.

---

## Step 6 — Curated spawner (don't spawn the whole field)

The library's `placeGamePiecesOnField()` for a real season often spawns **hundreds** of pieces
(2026 Rebuilt: ≥ 360), which is needless physics load for interactive sim. Write a
`GamePieceSpawner` that clears and places a **curated 20–40** in a layout that *mirrors* the
real field (dense center cluster + small depot clusters near the walls). Pattern, verbatim from
the 2026 work:

```java
public static void spawnInitialPieces(SimulatedArena arena) {
  arena.clearGamePieces();
  for (double[] p : PIECE_POSITIONS) {           // curated {x, y} list, metres
    arena.addGamePiece(new FuelOnField(new Translation2d(p[0], p[1])));
  }
}
```
Keep every position clear of obstacle footprints, walls, and field edges. Group with comments
(`// center 5×6 grid`, `// blue-wall depot`). See `GamePieceSpawner.FUEL_POSITIONS`.

> **"Reset Field" caveat:** the Glass DriverStation NT "Reset Field" widget calls the arena's
> `placeGamePiecesOnField()` / `resetFieldForAuto()`, which restores the **library** default
> layout (the big grid), not your curated set — unless your custom arena's
> `placeGamePiecesOnField()` override delegates to the spawner (Step 4). Note this for the user.

---

## Step 7 — Arena patch (only when using a library arena with bugs)

If you're using a **library** arena (Step 2, first branch) and Step 1 found a discrepancy
(missing/duplicated/mis-placed collision body), don't fork the library — add a reflective,
**idempotent, self-disabling** patch like `Arena2026Patch`:

- reach the `protected final World<Body> physicsWorld` field by reflection,
- scan existing bodies; **return early if the body already exists** (so it no-ops once the
  library is fixed),
- otherwise add a dyn4j `Body` (`MassType.INFINITE`, friction 0.6, restitution 0.3) at the
  correct transform via `GeometryConvertor.toDyn4jTransform(pose)`,
- guard the whole thing with `arena.getClass().getName().contains("Arena<Year><Name>")`.

Add a focused unit test mirroring `Arena2026PatchTest` (count bodies before/after; assert
idempotency).

---

## Step 8 — Wire the arena selection into the factory

In `SwerveFactory.buildWithPhysicsSim(...)`, **before** the first `SimulatedArena.getInstance()`
use, install the custom arena (only when the library has none):

```java
// New season, IronMaple arena not shipped yet → use our hand-built one.
SimulatedArena.overrideInstance(new Arena<Year><Name>());

SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSim);
// Arena<Year>Patch.applyMissing...();   // only if using a LIBRARY arena (Step 7)
GamePieceSpawner.spawnInitialPieces(SimulatedArena.getInstance());
```

If you're patching a library arena instead, drop the `overrideInstance` line and keep the
patch + spawner calls (this is the current 2026 wiring in `SwerveFactory.java:120-124`).

**Singleton teardown (tests):** every Layer 3 `@AfterEach` must `shutDown()` the arena and null
the static `instance` field by reflection (see CLAUDE.md "SimulatedArena singleton"). If you
inject via `overrideInstance`, the same null-the-field teardown still applies — the next test
must get a fresh arena.

---

## Step 9 — Draw the 2D field in the web view (`index.html`)

The browser field view is rendered on a `<canvas>` in `src/main/resources/web/index.html`. It
is **independent** of IronMaple — it draws from hard-coded geometry (matching your inventory)
plus live piece positions polled from `/api/gamepieces`. Reuse the existing helpers; do not
reinvent them:

| Helper (already in `index.html`) | Use it for |
|---|---|
| `drawField()` letterbox block | Preserves true field aspect; **don't stretch**. Computes `W,H,offX,offY` and `ctx.translate(offX,offY)`. |
| `fieldPt(fx, fy, W, H)` | Map a field metre point → canvas px (handles the Blue/Red alliance flip). |
| `fieldRect(ctx, fx, fy, dx, dy, …)` | Draw an axis-aligned obstacle rectangle (maps 4 corners). |
| `fieldHexagon(ctx, fx, fy, R, …)` | Draw a hex obstacle (hub-style). |
| `fieldAngle(rad)` | Convert a field heading → canvas rotation (for tag facing ticks, etc.). |
| `mToPx(m, W, H)` | Approx metres→px for label/marker sizing. |
| `fieldToCanvas(W, H)` | Robot body placement + heading. |

To add the new field:

1. **Dimensions / zones:** set `state.fieldWidth/fieldHeight` defaults and update the
   `/api/state` JSON in `WebDriveController.handleState(...)` to emit the right field size.
   Update the alliance-zone fraction (`zoneFrac`) to your Step-1 boundary.
2. **Obstacles:** replace `drawObstacles(ctx, W, H)` body with your inventory — one
   `fieldRect`/`fieldHexagon`/`arc` per obstacle, Blue/Red mirrored. Keep the inch→metre `M`
   constant and comment each block with its inventory label.
3. **AprilTags:** regenerate the `aprilTags[]` JS array from the season JSON (Step 1F) and drop
   the per-tag PNGs into `src/main/resources/web/tags/AT<id>.png` (served by `handleTagImage`).
4. **Game pieces:** `drawGamePieces(ctx, W, H)` already renders `gamePieces` (grounded) and
   `flyingPieces` (in-flight) from `/api/gamepieces`. Only change the colour/size to match the
   piece. The `/api/gamepieces` endpoint (`WebDriveController.handleGamePieces`) filters by
   `piece.getType()` — update the type string to your piece's `type`.
5. **Sidebar overlay:** the field is portrait, so any landscape window pillar-boxes it. The
   Held/Scored sidebar is an **absolute overlay over the left dark bar** (`#field-sidebar`,
   `position:absolute; pointer-events:none`) so the canvas spans the full width and the field
   stays **centered in the window**. Keep that pattern — do not put the sidebar back into flex
   flow (it shifts the field off-center, especially when narrow).

---

## Step 10 — Verify

```bash
./gradlew compileJava                          # no errors
./gradlew test                                 # all existing tests still pass
xvfb-run ./gradlew simulateJava -PtestSim      # SimRobotProfile + your custom arena
#   → open the forwarded port 5800 URL
#   → field renders with correct proportions, obstacles, tags, and curated pieces
#   → drive the robot; intake/collect; fire and watch projectiles + scoring
```

Cross-check the **web obstacles against the physics obstacles**: drive the robot into each
drawn barrier — it must collide. A barrier you can drive through means the canvas rect and the
`FieldMap` obstacle disagree (a transcription error in one of them). This is the single best
end-to-end check that your inventory was applied consistently to both renderers.

---

## Step 11 — Keep code, tests, and docs in sync (contribution rules)

Per `CLAUDE.md` "Contribution rules":

1. Full `./gradlew test` green (add Layer 3 tests for the arena/patch if you wrote one).
2. Update `docs/simulation.md` with the new arena + how to select it.
3. Add the new files to the **"Key file locations"** table in `CLAUDE.md`.
4. Add this command to the **"Slash commands available"** list in `CLAUDE.md` and `docs/`.
5. If the season later gets a real IronMaple arena, reduce your custom arena to an
   `Arena<Year>Patch` and flip `SwerveFactory` from `overrideInstance(...)` back to the library
   `getInstance()` + patch.

---

## Reference: the 2026 Rebuilt implementation this skill generalizes

| Concern | Look at |
|---|---|
| Reflective obstacle patch (idempotent) | `org.frc5010.common.drive.swerve.Arena2026Patch` |
| Curated piece spawner | `org.frc5010.common.drive.swerve.GamePieceSpawner` |
| Projectile launch + ballistic speed + scoring callback | `org.frc5010.common.drive.swerve.DemoIntake` |
| Factory wiring (patch + spawn after `addDriveTrainSimulation`) | `SwerveFactory.buildWithPhysicsSim` (~L118-124) |
| Web field draw (letterbox, obstacles, tags, pieces, sidebar) | `src/main/resources/web/index.html` (`drawField`, `drawObstacles`, `drawGamePieces`) |
| `/api/gamepieces` + `/api/state` field-size JSON | `WebDriveController.handleGamePieces`, `handleState` |
| Singleton teardown for tests | `CLAUDE.md` → "SimulatedArena singleton — test isolation" |

### Gotchas carried over from the 2026 build
- **`SimulatedArena` is a static singleton** — inject custom arenas with `overrideInstance`
  *before* the first `getInstance()`; null the `instance` field in test teardown.
- **Values hidden behind `protected static`** (hub Z height, depot corners) — decode from
  `javap -p -c` static initializers and comment the source.
- **`@AutoLog` inner-class fields stay primitive `double`** — never convert to `Measure<>`.
- **Per-cycle order in Layer 3 tests:** `runVelocity` → `simulationPeriodic` → `periodic`.
  Game-piece reads (`gamePiecesOnField()`) only reflect motion after `simulationPeriodic()`.
- **Keep the field's true aspect ratio** in the web view; letterbox, never horizontally stretch.
