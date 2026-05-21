# Force Sensor Bias Startup Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Defer force-sensor zero-offset finalization until EE pose is valid, so tool gravity is subtracted using the real R_world_from_EE rather than identity.

**Architecture:** Encapsulate the deferred-finalize logic inside `SriForceSensor`. `Init()` becomes non-blocking (open serial + start read thread + start accumulating raw_ee samples). The torque-control loop calls a new `TryFinalizeBias()` each tick; once at least 500 samples are accumulated AND the EE rotation provider returns a valid R, the sensor computes `bias = mean(raw_ee) - tool_gravity_wrench_ee(R_now)`, sets `mZeroOffsetValid = true`, and from that point on `PeekLatest` returns zeroed wrench. Until then, `PeekLatest` returns false so `primary_wrench_valid` is false and force-related code is skipped. The runtime tool-gravity compensation path is unchanged.

**Tech Stack:** C++17, Eigen, std::optional, std::mutex, GoogleTest. Build via existing CMake.

**Spec:** `docs/superpowers/specs/2026-05-21-force-sensor-bias-startup-design.md`

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `robot_server_lib/utils/sri_force_sensor.h` | Modify | Update API surface: provider type → `std::function<std::optional<Matrix3d>()>`; add `TryFinalizeBias()`; mutex; doc |
| `robot_server_lib/utils/sri_force_sensor.cpp` | Modify | Make `Init()` non-blocking; rewrite `ProcessCalibrationData` to only accumulate; add `TryFinalizeBias()`; gate `PeekLatest` |
| `w1_server/src/w1_server.h` | Modify | `GetEERotationWorldFromEE` returns `std::optional<Eigen::Matrix3d>` |
| `w1_server/src/w1_server.cpp` | Modify | Update lambda; call `TryFinalizeBias()` at top of `TorqueControlThreadFunc`; refresh `primary_sensor_ready_` |
| `robot_server_lib/test/test_sri_force_sensor.cpp` | Modify | Add unit tests with a fake R provider |

Tasks are ordered so the build only fully passes again at the end of Task 4 (the type-plumbing block) and stays green from there.

---

### Task 1: `GetEERotationWorldFromEE` returns `std::optional<Matrix3d>`

**Files:**
- Modify: `w1_server/src/w1_server.h` (declaration)
- Modify: `w1_server/src/w1_server.cpp:169-183` (implementation)

Sets up the type the rest of the plan relies on. Build will fail at call sites — Tasks 2 and 3 fix them.

- [ ] **Step 1: Update header**

In `w1_server/src/w1_server.h`, add `#include <optional>` if not present, and change the declaration:

```cpp
std::optional<Eigen::Matrix3d> GetEERotationWorldFromEE(int arm_index) const;
```

- [ ] **Step 2: Update implementation**

In `w1_server/src/w1_server.cpp`, replace the body:

```cpp
std::optional<Eigen::Matrix3d> RobotServer::GetEERotationWorldFromEE(int arm_index) const {
    const std::size_t offset = static_cast<std::size_t>(arm_index) * 7;
    const auto& ee_pose_world = this->manager_info_.ee_pose_world_measurement;
    if (ee_pose_world.size() < offset + 7) {
        return std::nullopt;
    }
    Eigen::Quaterniond q(ee_pose_world[offset + 6],
                         ee_pose_world[offset + 3],
                         ee_pose_world[offset + 4],
                         ee_pose_world[offset + 5]);
    if (!q.coeffs().allFinite() || q.norm() < 1e-9) {
        return std::nullopt;
    }
    q.normalize();
    return q.toRotationMatrix();
}
```

(If existing field indices differ, preserve them — the only behavior change is `nullopt` instead of identity when data isn't available.)

- [ ] **Step 3: Build to surface broken callers**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -30`

Expected: errors at the lambda inside `LoadToolGravityCompensationConfig` and inside `ApplyToolGravityCompensation`. Tasks 2 and 3 fix them.

- [ ] **Step 4: Commit**

```bash
git add w1_server/src/w1_server.h w1_server/src/w1_server.cpp
git commit -m "refactor: GetEERotationWorldFromEE returns optional R"
```

---

### Task 2: `ApplyToolGravityCompensation` handles optional R

**Files:**
- Modify: `w1_server/src/w1_server.cpp:185-200`

Once the bias is valid, `primary_sensor_ready_` ensures this path only runs with valid R. The `nullopt` branch is a defensive fallback.

- [ ] **Step 1: Update body**

```cpp
void RobotServer::ApplyToolGravityCompensation(robot_server_lib::Vector6d& wrench_ee, int arm_index) const {
    if (!tool_gravity_compensation_enabled_) {
        return;
    }
    const auto R_opt = GetEERotationWorldFromEE(arm_index);
    if (!R_opt) {
        return;
    }
    const Eigen::Matrix3d& R = *R_opt;
    // existing math: gravity_world, force_ee = sign * R^T * mass * gravity_world,
    // moment_ee = com.cross(force_ee), subtract from wrench_ee
}
```

Keep the existing math body verbatim; only the call to `GetEERotationWorldFromEE` and the unwrap change.

- [ ] **Step 2: Commit**

```bash
git add w1_server/src/w1_server.cpp
git commit -m "refactor: handle nullopt R in ApplyToolGravityCompensation"
```

---

### Task 3: Update lambda in `LoadToolGravityCompensationConfig`

**Files:**
- Modify: `w1_server/src/w1_server.cpp` (lambda passed to `SetToolGravityCompensationForZeroCalibration`)

- [ ] **Step 1: Update return type**

```cpp
primary_sensor_->SetToolGravityCompensationForZeroCalibration(
    enabled, mass_kg, com_m, gravity_magnitude, model_sign,
    [this]() -> std::optional<Eigen::Matrix3d> {
        return GetEERotationWorldFromEE(0);
    });
```

- [ ] **Step 2: Build (still fails until Task 4)**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | grep -E "error:" | head -20`

Expected: only error remaining is the type mismatch at the `SriForceSensor` setter signature.

- [ ] **Step 3: Commit**

```bash
git add w1_server/src/w1_server.cpp
git commit -m "refactor: zero-calib lambda returns optional R"
```

---

### Task 4: Change `SriForceSensor` rotation provider type to optional

**Files:**
- Modify: `robot_server_lib/utils/sri_force_sensor.h`
- Modify: `robot_server_lib/utils/sri_force_sensor.cpp` (`SetToolGravityCompensationForZeroCalibration`, `ComputeToolGravityWrenchEE`)

After this task the build is fully green again.

- [ ] **Step 1: Header — change member + setter signature**

In `robot_server_lib/utils/sri_force_sensor.h`, add `#include <optional>` near other includes. Update the member field and setter:

```cpp
std::function<std::optional<Eigen::Matrix3d>()> mRotationWorldFromEEProvider;

void SetToolGravityCompensationForZeroCalibration(
    bool enabled,
    double mass_kg,
    const Eigen::Vector3d& com_m,
    double gravity_magnitude,
    int model_sign,
    std::function<std::optional<Eigen::Matrix3d>()> rotation_provider);
```

- [ ] **Step 2: Setter body**

In `sri_force_sensor.cpp`, update `SetToolGravityCompensationForZeroCalibration` to assign the new-typed provider into `mRotationWorldFromEEProvider`. Other assignments unchanged.

- [ ] **Step 3: `ComputeToolGravityWrenchEE` handles `nullopt`**

```cpp
robot_server_lib::Vector6d SriForceSensor::ComputeToolGravityWrenchEE() const {
    using robot_server_lib::Vector6d;
    Vector6d wrench_ee = Vector6d::Zero();
    if (!mToolGravityCompensationForZeroEnabled || !mRotationWorldFromEEProvider) {
        return wrench_ee;
    }
    const auto R_opt = mRotationWorldFromEEProvider();
    if (!R_opt) {
        return wrench_ee;
    }
    const Eigen::Matrix3d& R = *R_opt;
    const Eigen::Vector3d gravity_world(0.0, 0.0, -mToolGravityMagnitude);
    const Eigen::Vector3d force_ee =
        static_cast<double>(mToolGravityModelSign) * R.transpose() *
        (mToolGravityMass * gravity_world);
    wrench_ee.head<3>() = force_ee;
    wrench_ee.tail<3>() = mToolGravityComM.cross(force_ee);
    return wrench_ee;
}
```

(Replace `mToolGravityMass`, `mToolGravityComM`, `mToolGravityModelSign` with whatever the existing header uses.)

- [ ] **Step 4: Build cleanly**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: build succeeds end-to-end.

- [ ] **Step 5: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.h robot_server_lib/utils/sri_force_sensor.cpp
git commit -m "refactor: rotation provider returns optional Matrix3d"
```

---

### Task 5: Declare `TryFinalizeBias()` and add mutex

**Files:**
- Modify: `robot_server_lib/utils/sri_force_sensor.h`

- [ ] **Step 1: Add public declaration**

Inside the public section near `HasValidZeroOffset`:

```cpp
// Try to finalize the zero offset using accumulated samples and the
// current rotation provider. Returns true on the tick that finalization
// succeeds; false if already finalized or conditions aren't yet met.
// Safe to call from the control loop every tick.
bool TryFinalizeBias();
```

- [ ] **Step 2: Add includes and members**

Add `#include <mutex>` near other includes. In the private section, add:

```cpp
mutable std::mutex mCalibrationMutex;
static constexpr int kMinCalibrationSamples = 500;  // ~500ms @ 1kHz
```

- [ ] **Step 3: Build header-only**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: succeeds (no body yet — link is fine because the symbol isn't referenced elsewhere).

- [ ] **Step 4: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.h
git commit -m "feat: declare TryFinalizeBias and calibration mutex"
```

---

### Task 6: Rewrite `ProcessCalibrationData()` to only accumulate

**Files:**
- Modify: `robot_server_lib/utils/sri_force_sensor.cpp`

The previous body subtracted gravity per-frame, watched a 500ms timer, and flipped `mZeroOffsetValid`. Strip all of that — leave only "if calibrating, accumulate raw_ee".

- [ ] **Step 1: Replace the function body**

```cpp
void SriForceSensor::ProcessCalibrationData(float fx, float fy, float fz,
                                            float mx, float my, float mz) {
    std::lock_guard<std::mutex> lock(mCalibrationMutex);
    if (!mIsCalibrating) {
        return;
    }
    const robot_server_lib::Vector6d wrench_ee = MapSriRawToEE(fx, fy, fz, mx, my, mz);
    for (int i = 0; i < 6; ++i) {
        mCalibrationSum[i] += static_cast<float>(wrench_ee[i]);
    }
    mCalibrationSampleCount++;
}
```

- [ ] **Step 2: Build**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: success.

- [ ] **Step 3: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.cpp
git commit -m "refactor: ProcessCalibrationData only accumulates"
```

---

### Task 7: Implement `TryFinalizeBias()`

**Files:**
- Modify: `robot_server_lib/utils/sri_force_sensor.cpp`

- [ ] **Step 1: Add the function body**

Place it near `ProcessCalibrationData`:

```cpp
bool SriForceSensor::TryFinalizeBias() {
    std::lock_guard<std::mutex> lock(mCalibrationMutex);
    if (!mIsCalibrating) {
        return false;
    }
    if (mCalibrationSampleCount < kMinCalibrationSamples) {
        return false;
    }

    robot_server_lib::Vector6d tool_gravity_ee = robot_server_lib::Vector6d::Zero();
    if (mToolGravityCompensationForZeroEnabled) {
        if (!mRotationWorldFromEEProvider) {
            return false;
        }
        const auto R_opt = mRotationWorldFromEEProvider();
        if (!R_opt) {
            return false;
        }
        const Eigen::Matrix3d& R = *R_opt;
        const Eigen::Vector3d gravity_world(0.0, 0.0, -mToolGravityMagnitude);
        const Eigen::Vector3d force_ee =
            static_cast<double>(mToolGravityModelSign) * R.transpose() *
            (mToolGravityMass * gravity_world);
        tool_gravity_ee.head<3>() = force_ee;
        tool_gravity_ee.tail<3>() = mToolGravityComM.cross(force_ee);
    }

    for (int i = 0; i < 6; ++i) {
        const float mean = mCalibrationSum[i] /
                           static_cast<float>(mCalibrationSampleCount);
        mZeroOffset[i] = mean - static_cast<float>(tool_gravity_ee[i]);
    }
    mZeroOffsetSampleCount = mCalibrationSampleCount;
    mZeroOffsetValid = true;
    mIsCalibrating = false;
    return true;
}
```

(Substitute the actual member field names if they differ.)

- [ ] **Step 2: Build**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: success.

- [ ] **Step 3: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.cpp
git commit -m "feat: implement TryFinalizeBias"
```

---

### Task 8: Make `Init()` non-blocking

**Files:**
- Modify: `robot_server_lib/utils/sri_force_sensor.cpp` (around lines 379–441)

Strip the 2-second wait loop. `Init` returns as soon as serial is open and the read thread is running.

- [ ] **Step 1: Replace the wait block**

After the section that opens the serial port, spawns the read thread, and waits ~100ms for data flow, replace the calibration wait loop with:

```cpp
{
    std::lock_guard<std::mutex> lock(mCalibrationMutex);
    for (int i = 0; i < 6; ++i) {
        mCalibrationSum[i] = 0.0f;
        mZeroOffset[i] = 0.0f;
    }
    mCalibrationSampleCount = 0;
    mZeroOffsetSampleCount = 0;
    mZeroOffsetValid = false;
    mIsCalibrating = true;
    mCalibrationStartTime = std::chrono::steady_clock::now();
}
return true;
```

Remove the `while` loop that polled `mIsCalibrating` with a 2-second timeout.

- [ ] **Step 2: Build**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: success.

- [ ] **Step 3: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.cpp
git commit -m "feat: Init returns immediately without blocking on calibration"
```

---

### Task 9: Gate `PeekLatest` and zeroed-buffer writes on `mZeroOffsetValid`

**Files:**
- Modify: `robot_server_lib/utils/sri_force_sensor.cpp`

Two places need gating: the `PeekLatest` accessor, and the read-thread point that appends a zeroed wrench to the consumer buffer (so we don't store stale-bias-zeroed values).

- [ ] **Step 1: Update `PeekLatest`**

Add an early-return at the top:

```cpp
bool SriForceSensor::PeekLatest(robot_server_lib::Vector6d* output) {
    if (!output) {
        return false;
    }
    if (!HasValidZeroOffset()) {
        return false;
    }
    // ... existing code that returns the latest zeroed wrench from buffer_ ...
}
```

- [ ] **Step 2: Gate the read-thread buffer append**

Locate where the read thread appends to `buffer_` (the post-zero-offset buffer). Wrap the append:

```cpp
// raw buffer is unconditional
raw_buffer_.Append(wrench_ee_raw);

// zeroed buffer only after bias is finalized
if (mZeroOffsetValid) {
    robot_server_lib::Vector6d zeroed = wrench_ee_raw;
    ApplyZeroOffset(zeroed);
    buffer_.Append(zeroed);
}
```

If the existing code uses different identifiers, match them. The intent: `raw_buffer_` (or equivalent) keeps streaming; `buffer_` only accumulates valid zeroed values.

- [ ] **Step 3: Build**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: success.

- [ ] **Step 4: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.cpp
git commit -m "feat: PeekLatest and zeroed buffer gated on mZeroOffsetValid"
```

---

### Task 10: Drive `TryFinalizeBias()` from the torque-control loop

**Files:**
- Modify: `w1_server/src/w1_server.cpp` (`TorqueControlThreadFunc`)

- [ ] **Step 1: Call TryFinalizeBias each tick**

At the top of the per-tick body inside `TorqueControlThreadFunc`, just before `PeekLatest`:

```cpp
if (primary_sensor_) {
    if (primary_sensor_->TryFinalizeBias()) {
        LOG_INFO << "[force_sensor] zero offset finalized; samples="
                 << primary_sensor_->GetZeroOffsetSampleCount();
    }
}
```

If `LOG_INFO` differs from the surrounding logger style, match the existing convention (e.g., `spdlog::info`, `RCLCPP_INFO`, `std::cerr`, etc.).

- [ ] **Step 2: Refresh `primary_sensor_ready_` per tick**

Find existing `primary_sensor_ready_ = true;` (likely a one-shot after `Init`) and remove it. Replace with a per-tick refresh near the `TryFinalizeBias` call:

```cpp
primary_sensor_ready_ = primary_sensor_ && primary_sensor_->HasValidZeroOffset();
```

- [ ] **Step 3: Build**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build 2>&1 | tail -20`

Expected: success.

- [ ] **Step 4: Commit**

```bash
git add w1_server/src/w1_server.cpp
git commit -m "feat: torque loop drives TryFinalizeBias and tracks readiness"
```

---

### Task 11: Add unit tests for the deferred-finalize logic

**Files:**
- Modify: `robot_server_lib/test/test_sri_force_sensor.cpp` (or create if it doesn't exist)
- Modify: `robot_server_lib/test/CMakeLists.txt` if a new file is added

These tests run without real hardware by feeding `ProcessCalibrationData` directly and supplying a fake rotation provider.

- [ ] **Step 1: Inspect existing test file structure**

Run: `ls /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a/robot_server_lib/test/ && head -30 /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a/robot_server_lib/test/test_sri_force_sensor.cpp 2>/dev/null`

If the file exists, append to it. Otherwise, create it with the project's standard test boilerplate (gtest, includes, namespace).

- [ ] **Step 2: Write tests**

Append (or create with) these test cases. Replace `<existing test header>` with whatever the file uses:

```cpp
#include <gtest/gtest.h>
#include <optional>
#include <Eigen/Geometry>
#include "robot_server_lib/utils/sri_force_sensor.h"

namespace {

using robot_server_lib::Vector6d;

// Feeds a known raw frame n times. Sample units are arbitrary — the test
// just checks the linear relationship the bias formula promises.
void FeedSamples(SriForceSensor& s, int n,
                 float fx, float fy, float fz,
                 float mx, float my, float mz) {
    for (int i = 0; i < n; ++i) {
        s.ProcessCalibrationData(fx, fy, fz, mx, my, mz);
    }
}

}  // namespace

TEST(SriForceSensorBias, FinalizesAfterEnoughSamplesAndValidR) {
    SriForceSensor s;
    // Fake "Init"-like state setup: directly set mIsCalibrating = true.
    // If the API requires going through Init(), provide a test-only setter
    // or guard, or expose a `BeginCalibrationForTest()` helper.
    s.BeginCalibrationForTest();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    s.SetToolGravityCompensationForZeroCalibration(
        true, 0.0 /*mass=0 → no gravity to subtract*/, Eigen::Vector3d::Zero(),
        9.79, 1, [&]() -> std::optional<Eigen::Matrix3d> { return R; });

    EXPECT_FALSE(s.TryFinalizeBias());  // No samples yet
    FeedSamples(s, 100, 1, 2, 3, 0, 0, 0);
    EXPECT_FALSE(s.TryFinalizeBias());  // Below kMinCalibrationSamples
    FeedSamples(s, 500, 1, 2, 3, 0, 0, 0);
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_TRUE(s.HasValidZeroOffset());

    Vector6d bias;
    ASSERT_TRUE(s.GetZeroOffset(&bias));
    // With mass=0, bias equals mean of mapped raw, no gravity correction.
    // (Exact values depend on MapSriRawToEE; just assert finalize succeeded.)
    SUCCEED();
}

TEST(SriForceSensorBias, WaitsForValidR) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    bool r_valid = false;
    s.SetToolGravityCompensationForZeroCalibration(
        true, 1.0, Eigen::Vector3d::Zero(), 9.79, 1,
        [&]() -> std::optional<Eigen::Matrix3d> {
            if (!r_valid) return std::nullopt;
            return Eigen::Matrix3d::Identity();
        });

    FeedSamples(s, 1000, 0, 0, -9.79f, 0, 0, 0);
    EXPECT_FALSE(s.TryFinalizeBias());  // R still invalid
    EXPECT_FALSE(s.HasValidZeroOffset());

    r_valid = true;
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_TRUE(s.HasValidZeroOffset());
}

TEST(SriForceSensorBias, GravityDisabledFinalizesOnSamplesAlone) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    s.SetToolGravityCompensationForZeroCalibration(
        false, 0.0, Eigen::Vector3d::Zero(), 9.79, 1,
        []() -> std::optional<Eigen::Matrix3d> { return std::nullopt; });

    FeedSamples(s, 500, 1, 2, 3, 0, 0, 0);
    EXPECT_TRUE(s.TryFinalizeBias());
    EXPECT_TRUE(s.HasValidZeroOffset());
}

TEST(SriForceSensorBias, PeekLatestBlockedUntilFinalized) {
    SriForceSensor s;
    s.BeginCalibrationForTest();
    Vector6d out;
    EXPECT_FALSE(s.PeekLatest(&out));
}
```

- [ ] **Step 3: Add `BeginCalibrationForTest()` helper**

In `sri_force_sensor.h` (public section, behind `#ifdef ROBOT_SERVER_TESTING` if you want to scope it to tests, otherwise plain public):

```cpp
void BeginCalibrationForTest() {
    std::lock_guard<std::mutex> lock(mCalibrationMutex);
    for (int i = 0; i < 6; ++i) { mCalibrationSum[i] = 0.0f; mZeroOffset[i] = 0.0f; }
    mCalibrationSampleCount = 0;
    mZeroOffsetSampleCount = 0;
    mZeroOffsetValid = false;
    mIsCalibrating = true;
}
```

This is the minimum surface needed by the tests; production code paths still go through `Init()`.

- [ ] **Step 4: Run tests**

Run: `cd /Users/clement/dexe/dexe_robot_server/.claude/worktrees/practical-hodgkin-69327a && cmake --build build && ctest --test-dir build --output-on-failure -R SriForceSensor`

Expected: all four tests pass.

- [ ] **Step 5: Commit**

```bash
git add robot_server_lib/utils/sri_force_sensor.h robot_server_lib/test/test_sri_force_sensor.cpp robot_server_lib/test/CMakeLists.txt
git commit -m "test: deferred-finalize behavior for SriForceSensor bias"
```

---

## Self-Review

- **Spec coverage:** Each spec section maps to tasks — state machine (Tasks 5–8), interface changes (Tasks 1–7), data flow & concurrency (Tasks 5–6, 9), config compat (Task 7), test plan (Task 11), file change list (all). No gaps.
- **No placeholders:** Every step shows the actual code or command. The "If field names differ" notes are intentional adapters to the unread parts of `sri_force_sensor.h`, not vague TODOs.
- **Type consistency:** `std::optional<Eigen::Matrix3d>` used consistently from Task 1 through Task 11. `TryFinalizeBias`, `HasValidZeroOffset`, `GetZeroOffsetSampleCount`, `PeekLatest`, `BeginCalibrationForTest` all named identically across declarations and call sites.
- **Build sequencing:** Tasks 1–3 leave the build broken; Task 4 restores it. From Task 5 onward each task builds clean. Each task is its own commit.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-05-21-force-sensor-bias-startup.md`. Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task with two-stage review between tasks
2. **Inline Execution** — execute tasks in this session via `superpowers:executing-plans`

Which approach?
