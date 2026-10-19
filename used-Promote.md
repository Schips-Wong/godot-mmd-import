## v1

### a

这个是一个Godot4插件工程，实现了.pmx（MMD模型）模型的导入。

帮我继续实现.vmd文件的导入（MMD模型动作）为动画数据。

相关的参考：
- Blender导入mmd相关格式的插件：[blender_mmd_tools](https://github.com/MMD-Blender/blender_mmd_tools)
- ​Godot源码：D:\godot\godot
- Godot文档：D:\godot\godot-docs

### b

我新增了一个test场景，经过测试，动作的播放有问题，部分骨骼没有被驱动起来。

帮我检查一下是pmx导入的问题还是vmd导入的问题

注意，在Blender里面动作是正常的。

```
检查完了：**PMX 导入和 VMD 导入都没有问题**，问题在于缺少 MMD 的两条运行时骨骼规则：**IK** 和 **付与（append / grant）**。Blender 的 mmd_tools 用 IK 约束和 Transform 约束实现了这两条规则，所以那边看起来正常。

## 证据

**1. 这个 VMD 中真正有动作的骨骼只有 17 根**（其余 1440+ 关键帧全是 1 帧恒定或全零）：

    センター, 上半身, 上半身2, 下半身, 首, 頭, 両目,
    右肩/右腕/右ひじ/右手首, 左肩/左腕/左ひじ/左手首,
    右足ＩＫ, 左足ＩＫ


**2. 腿完全由 IK 驱动，VMD 里没有腿的旋转**

`右足`、`右ひざ`、`右足首` 在 VMD 里**各只有 1 个关键帧**（单位旋转、零位移），轨道值正确写进了动画，但动画本身就应当保持不动。真正驱动它们的是 IK 骨骼：

    PMX: 右足ＩＫ -> target=右足首, 40 次迭代
        link 右ひざ  限位 X: [-180°, -0.5°]   ← 膝盖只能绕 X 弯
        link 右足    无限位
        右つま先ＩＫ -> target=右つま先, link=右足首
    右足ＩＫ rest 位置 = 右足首 rest 位置（完全重合，符合 MMD 规范）


实测播放到第 700 帧时，**IK 目标和脚踝相距 0.215 m**（第 300 帧 0.62 m）；也就是说目标在移动，脚却没有被拉过去，腿全程保持并跟随胯部僵硬移动。

**3. 扭转和辅助骨骼依赖“付与”（append）**

    右腕捩1 <- 右腕捩 权重 0.25    右肩C  <- 右肩P 权重 -1.0（取反）
    右腕捩2 <- 右腕捩 权重 0.50    右ひじ補助 <- 右ひじ 0.6
    右腕捩3 <- 右腕捩 权重 0.75    右目戻 <- 右目 -0.7
    右足D   <- 右足   权重 1.0     右ひざD <- 右ひざ 1.0   ← 网格蒙皮用到 D 骨

这些骨骼在 VMD 里同样只有 1 帧单位旋转。实测第 700 帧时：`右腕` 转了 50.5°，而 `右腕捩1/2/3`、`右肩C` 全是 **0°**。因此手臂不扭转、肩 C 不跟随，而 `右足D/右ひざD` 不跟随还会导致腿部网格局部不动。

**4. 两个导入器的输出经核对是正确的**

- PMX：511 根骨骼、无重名、父级全部合法、rest 位置与 IK 定义与文件一致；
- VMD：轨道数值已与独立的 Python 实现（CP932 + VMD + slerp）逐帧比对一致，`上半身` 第 300 帧 `(0,0,0.1296,0.9916)`、`センター` 第 600 帧高度 `0.67775` 全部吻合。

## 需要补的部分

MMD 的姿态解算属于运行时逻辑，无法用普通 Animation 轨道表达，建议新增：

1. `MMDPoseSolver extends SkeletonModifier3D`（Godot 4.3+，自动在 AnimationMixer 之后执行）：
   - 按变形顺序遍历：① 动画姿态 → ② **付与**：`q *= slerp(单位, q_目标, 权重)`（负权重为取反，位移同理）→ ③ **IK**：CCD，按 PMX 的迭代次数和逐轴限位，先 `足ＩＫ` 后 `つま先ＩＫ`；
2. PMX 导入器把每根骨骼的 MMD 数据（付与父骨/权重/标志、IK target/links/限位/迭代、变形顺序）写入 `Skeleton3D` 的 metadata，并自动挂上该节点；
3. VMD 导入器不用改。

一点提醒：这个 VMD 给 `右足ＩＫ/左足ＩＫ` 写入了恒定位移 ±2.2（22 cm 外扩），它原本是给 Tda式ミク 的。实现 IK 后站姿会比 琪亚娜 模型默认更宽，这是动作数据本身，不是 bug。

需要我现在就按这个方案实现 `MMDPoseSolver` 吗？如果你更希望**导入时烘焙**成纯 Animation（不带运行时节点、但动画体积会到几 MB），也可以改成那种方案。
```

### c

部分pmx模型无法导入。

提示：
ERROR: res://addons/mmd-import/pmx_importer.gd:128 - Out of bounds get index '-1' (on base: 'Dictionary[int, StandardMaterial3D]')
ERROR: core/io/file_access.cpp:1002 - Condition "f.is_null()" is true. Continuing.

```
已定位并修复。这个崩溃与元数据常量无关，是 `pmx_importer.gd` 里材质/贴图映射的既有缺陷，只是修改导入器脚本会让 Godot 重新导入所有 `.pmx`，从而把它暴露出来。

原因：
- PMX 中无贴图材质的 `m_textureIndex` 是 `-1`（`pmx_file.gd` 的 `get_index` 把 `0xFF/0xFFFF` 解码为 `-1`），而 `textureMap[...]` 直接取值，触发 `Out of bounds get index '-1'`。
- 旧代码遇到名字以 `0Nub` 结尾的贴图会 `continue`，导致 `textureMap` 出现空洞，被引用时同样会崩。
- 第二行 `file_access.cpp ... f.is_null()` 是对不存在的贴图文件调用 `load()` 造成的报错。

修改（`addons/mmd-import/pmx_importer.gd`）：
- 每个贴图索引都建立材质条目，`0Nub` 只跳过加载贴图、不再跳过建表。
- `load()` 前用 `FileAccess.file_exists()` 判断，避免缺失贴图刷错误。
- 取材质改为 `textureMap.has(...)` 判断，`-1` 时回退到默认材质。
```

### d

对于部分骨骼是采用IK解算的，要不你把这个部分降为普通的动画帧

### e

为 IK link 骨和付与骨建轨道，付与含位移时额外建 POSITION_3D 轨。

让`IK 和付与（append）现在都能在导入时烘焙成普通动画帧，并且可以完全不依赖运行时节点`。

```
已完成。IK 和付与（append）现在都能在导入时烘焙成普通动画帧，并且可以完全不依赖运行时节点。

## 改动

**新增 `addons/mmd-import/mmd_pose_baker.gd`**（替代原 `mmd_ik_baker.gd`）
- `bake(..., bake_append)`：逐帧采样 → 按 PMX 变形顺序执行「付与 → IK」→ 写键；
- 同时为 IK link 骨和付与骨建轨道，付与含位移时额外建 `POSITION_3D` 轨；
- 每帧重置未被 VMD 驱动的受规则影响骨骼，避免累积；
- 返回并打印烘焙骨骼数与帧数。

**`pmx_importer.gd`**
- 新增导入选项 `mmd/runtime_pose_solver`（默认 `true`）；
- 关闭后不创建 `MMDPoseSolver` 节点，但 `mmd_bone_data` 元数据仍写入（烘焙器需要）。

**`vmd_importer.gd`**
- 参考模型没有 `MMDPoseSolver` 节点时自动 `bake_append = true`，并打印提示；
- 有求解器时保持原行为（只烘焙 IK），避免付与被算两遍。

## 实测（临时工程 headless 重导入 `琪亚娜1.0.pmx` + `ankha.vmd`）

| | 默认（求解器开） | 关闭求解器 |
|---|---|---|
| 日志 | `baked 6 bones (付与 baked: false)` | `baked 34 bones (付与 baked: true)` |
| 动画 `.res` | 311 KB | **1.31 MB** |
| `右足D/右ひざD/右肩C/右腕捩1` | 各 1 键 | 各 1493 键 |
| 场景内 `MMDPoseSolver` | 有 | 无 |
| D 骨 vs 腿骨旋转差 | — | 0.00° / 0.04°（正确跟随 IK 后的腿） |

无 IK、无付与的模型不受影响，`bake` 会直接返回 0。

## 使用注意

1. 在 PMX 的导入面板取消 `MMD/runtime_pose_solver` 后，需要**手动重新导入对应的 VMD**——VMD 不会自动依赖 PMX。
2. 烘焙后的动作不要再加载 `MMDPoseSolver`，否则付与会叠加两次。
3. 该模式下所有动画都必须是烘焙过的；手工制作的 Animation 不会有付与效果。
```

使用注意是什么意思，我不是很明白

```
这两条说的是“切换烘焙模式时的两个坑”，不是报错，我拆开解释。

## 注意 1：切换 PMX 选项后，旧的 VMD 不会自动重烘焙

原因：付与是否烘焙，是 **VMD 导入那一刻**根据参考模型有没有 `MMDPoseSolver` 节点判断的。VMD 导入器只是临时 `load` 了 PMX 场景，Godot 并不会把 PMX 记成 VMD 的导入依赖（`.vmd.import` 的 `[deps]` 里只有它自己）。

所以会出现这种状态：

```
PMX：已关闭求解器   → 场景里没有 MMDPoseSolver
VMD：还是旧的 .res  → 付与没烘焙，靠求解器
```

结果就是 `右足D`、`右ひざD`、`右腕捩1`、`右肩C` 这些骨既不继承、也没有烘焙数据 → 腿/手臂的蒙皮形变会不对劲。

**做法**：改完 `.pmx` 的导入面板后，对每个 `.vmd` 执行一次 **Reimport**（资源面板右键 → Reimport，或在导入面板点“重新导入”）。日志里出现这两行就说明生效了：

```
MMD: reference model has no runtime pose solver, 付与 is baked into the motion
MMD: baked 34 bones over 1493 frames (付与 baked: true)
```

反过来，把选项改回开启时，也要再重导一次 VMD，日志会变回 `付与 baked: false`。

## 注意 2：烘焙后的动作不能再配求解器

付与本质是「子骨 = 自己的动画姿态 × 父骨带来的增量」。烘焙已经把这个增量乘进去了：

- `右腕捩1`：0.25 × 右腕
- `右肩C`：-1.0 × 右肩P（取反）
- `右足D`：1.0 × 右足

如果动画里已经写好了结果，运行时 `MMDPoseSolver` 又算一遍，就等于乘了两次：捩骨从 25% 变成约 50%，`右肩C` 的 -1 再乘一次会抵消回正向。

所以两种模式是二选一：

| | 模型带求解器 | 模型不带求解器 |
|---|---|---|
| VMD 烘焙 | 只烘焙 IK | IK + 付与 都烘焙 |
| 付与由谁算 | `MMDPoseSolver` 每帧算 | 动画数据本身 |
| 手工做的 Animation | 有付与效果 | **没有**付与效果 |

表格最后一行就是注意 3 的来源：不带求解器时，只有经过 `VMDImporter` 烘焙的动作才有付与；你自己在动画编辑器里手 K 的动画、或从别处导入的动画，捩骨/D 骨不会跟随。

## 建议的切换流程

1. 选中 `.pmx` → 导入面板 → 取消 `MMD/runtime_pose_solver` → 重新导入；
2. 对所有需要用的 `.vmd` 逐个 Reimport；
3. 打开导入后的模型检查 `Bone` 下没有 `MMDPoseSolver` 节点，动画里 `右足D` 等骨有 1493 帧键；
4. 想改回运行时方案就反向操作，并再次 Reimport VMD。

如果嫌第 2 步麻烦，可以保持默认（求解器开启 + 只烘焙 IK），效果与改动前完全一致；纯 Animation 模式适合确定要导出、不想带运行时节点的场景。
```