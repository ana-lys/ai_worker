# Xacro Block Parameter Bug (ROS 2 Jazzy)

## The Bug

xacro 2.1.1 in ROS 2 Jazzy does **not** expand `<insert_block name="origin"/>` when the macro
uses a `*origin` block parameter (`params="parent prefix *origin"`). The literal text
`<insert_block name="origin"/>` appears in the output XML instead of the passed block content.

This affects any macro that uses `*name` block parameters.

## Files Fixed (workaround applied)

Gripper macros converted from `*origin` block to plain `origin_xyz` + `origin_rpy` string params:

| File | Change |
|---|---|
| `ffw_description/urdf/common/xm430_gripper/xm430_gripper.urdf.xacro` | `*origin` → `origin_xyz origin_rpy` |
| `ffw_description/urdf/common/rh_p12_rn_a/rh_p12_rn_a.urdf.xacro` | `*origin` → `origin_xyz origin_rpy` |

Callers updated to pass `origin_xyz="..." origin_rpy="..."` instead of a `<origin>` block:

- `ffw_sg2_smtm/ffw_sg2_smtm.urdf.xacro` — left & right gripper
- `ffw_sg2_rev1_follower/ffw_sg2_follower.urdf.xacro` — left & right gripper
- `ffw_bg2_rev2_follower/ffw_bg2_follower.urdf.xacro` — left & right gripper
- `ffw_bg2_rev3_follower/ffw_bg2_follower.urdf.xacro` — left & right gripper
- `ffw_bg2_rev4_follower/ffw_bg2_follower.urdf.xacro` — left & right gripper

## Still Affected (not yet converted)

These body macros still use the broken `*origin` block parameter pattern and will not
receive their `<origin>` block content correctly:

- `ffw_description/urdf/common/follower/ffw_follower_body.xacro`
- `ffw_description/urdf/common/leader/ffw_leader_body.urdf.xacro`
- `ffw_description/urdf/ffw_bg2_rev3_follower/ffw_follower_body.xacro`

If body positioning ever needs to change, they need the same treatment.

## Workaround Pattern

**Before (broken in Jazzy):**

```xml
<xacro:macro name="example" params="parent *origin">
  <joint ...>
    <insert_block name="origin"/>
  </joint>
</xacro:macro>

<!-- caller -->
<xacro:example parent="base">
  <origin xyz="1 2 3" rpy="0 0 0"/>
</xacro:example>
```

**After (works in Jazzy):**

```xml
<xacro:macro name="example" params="parent origin_xyz origin_rpy">
  <joint ...>
    <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
  </joint>
</xacro:macro>

<!-- caller -->
<xacro:example parent="base"
  origin_xyz="1 2 3" origin_rpy="0 0 0"/>
```
