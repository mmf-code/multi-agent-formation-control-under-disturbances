# Configuration Options for Fair Controller Comparison

## 🔍 Current Problems

### Problem 1: Position Mismatch
**Drone spawn positions (world file):**
- Group 0 (Fuzzy): y = -6.0m
- Group 1 (PD): y = 0.0m
- Group 2 (PID): y = 6.0m

**Waypoint targets (config files):**
- Group 0: y = -4.0m ❌ (2m difference!)
- Group 1: y = 0.0m ✅
- Group 2: y = 4.0m ❌ (2m difference!)

**Result:** agent_6 has 21m error because it spawns at y=6.0 but target is y=4.0!

### Problem 2: Wind Direction
**Current wind:**
```xml
<x>0.1</x>
<y>1.0</y>  <!-- Wind in Y direction -->
```

**Drone movement:** X direction (east)

**Issue:** Wind is at an angle to movement, not perpendicular.

---

## ✅ Proposed Solutions

### Option 1: Separate on Z-axis (RECOMMENDED)

**Configuration:**
- All drones at **y = 0.0m** (same Y position)
- Separate on **Z-axis** (different altitudes):
  - Group 0 (Fuzzy): z = 2.0m
  - Group 1 (PD): z = 3.0m
  - Group 2 (PID): z = 4.0m
- **Wind:** Y-axis (perpendicular to X movement)
- **Movement:** All groups move in X direction

**Pros:**
- ✅ Wind perfectly perpendicular to flight path
- ✅ No collision risk (vertical separation)
- ✅ Simple trajectory (all same Y)
- ✅ Easy to visualize

**Cons:**
- ❌ Different altitudes = slightly different wind (but minimal at 1m intervals)

---

### Option 2: Separate on Y-axis (CURRENT, needs fix)

**Configuration:**
- All drones at **z = 3.0m** (same altitude)
- Separate on **Y-axis** with larger spacing:
  - Group 0 (Fuzzy): y = -12.0m
  - Group 1 (PD): y = 0.0m
  - Group 2 (PID): y = +12.0m
- **Wind:** X-axis (parallel to movement) OR Y-axis (perpendicular)
- **Movement:** All groups move in X direction

**Pros:**
- ✅ Same altitude = exactly same wind
- ✅ Large Y separation = no collision

**Cons:**
- ❌ More complex waypoint configuration
- ❌ Wind direction choice affects results

---

### Option 3: Diagonal Separation

**Configuration:**
- Group 0: y = -8m, z = 2m
- Group 1: y = 0m, z = 3m
- Group 2: y = +8m, z = 4m
- **Wind:** Configured for equal effect on all

**Pros:**
- ✅ Maximum separation
- ✅ Visually distinct

**Cons:**
- ❌ Complex to ensure equal wind effect

---

## 🎯 Recommendation

**Use Option 1 (Z-axis separation):**

1. **Simplest to implement**
2. **Wind perfectly perpendicular**
3. **No collision risk**
4. **Fair comparison** (1m altitude difference is negligible)

**Configuration:**
```yaml
Group 0 (Fuzzy): spawn(-15, 0, 2.0), waypoints(x: -8→20, y: 0, z: 2.0)
Group 1 (PD):    spawn(-10, 0, 3.0), waypoints(x: -6→20, y: 0, z: 3.0)
Group 2 (PID):   spawn(-5,  0, 4.0), waypoints(x: -2→20, y: 0, z: 4.0)
```

**Wind:**
```xml
<wind_direction>
  <x>0.0</x>
  <y>1.0</y>  <!-- Perpendicular to X movement -->
  <z>0.0</z>
</wind_direction>
```

---

## 📊 Expected Results

With Option 1:
- **Fuzzy (2m):** Best wind rejection (adaptive control)
- **PD (3m):** Fast settling, moderate wind handling
- **PID (4m):** Balanced, but slower adaptation

**All groups experience similar wind** (1m altitude difference ≈ 5% variation, acceptable)

---

## 🚀 Next Steps

**Choose an option and I will:**
1. Update world file (drone positions)
2. Update config files (waypoints)
3. Update wind direction
4. Test for collisions
5. Verify equal wind effect

**Which option do you prefer?**
