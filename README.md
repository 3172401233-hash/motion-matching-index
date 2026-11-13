# motion-matching-index

最小可用的 motion matching **数据与检索**原型：从一批 2D 骨骼动画片段构建逐帧特征索引，按「当前姿态 + 速度变化 + 未来轨迹」联合代价查询候选帧。不包含角色控制、混合或可视化。

## 快速开始

```js
import { buildMotionIndex, queryMotionIndex } from './src/index.js';

const index = buildMotionIndex(clips, {
  weights: { pose: 1, motion: 0.5, trajectory: 2 },
  scales: { 'velocity.x': 1.5, 'trajectory@0.5s.heading': 3.14159 },
});

const candidates = queryMotionIndex(index, queryFrame, {
  topK: 8,
  excludedClipIds: ['just-played-clip'],
});
```

运行测试：

```bash
node --test
```

## 输入数据格式

每个片段（clip）：

```js
{
  id: 'walk-forward',          // 唯一字符串 id
  loop: false,                 // 是否为循环片段
  frames: [ /* Frame，按 time 严格递增 */ ],
}
```

每帧（frame）：

```js
{
  time: 0.3,
  root: { x, y, heading },     // 必填，根节点位置与朝向（弧度）
  velocity: { x, y },          // 必填，世界空间速度；零向量合法
  feet: {                      // 可选；整个字段或单个脚均可缺失
    left:  { x, y } | null,
    right: { x, y } | null,
  },
  trajectory: [                // 可选；未来轨迹采样点
    { t, x, y, heading },      // t 为绝对时间（不是相对时间）
  ],
  // 加速度可选，提供任一种即可：
  acceleration: { x, y },      // 显式加速度
  prevVelocity: { x, y },      // 或上一帧速度（配合固定帧间隔差分）
  nextVelocity: { x, y },      // 或下一帧速度
}
```

构建时若未提供加速度，提取器按优先级使用：相邻帧速度前向/后向差分 → `nextVelocity`/`prevVelocity` 差分。都不可用时（例如单帧片段）加速度通道保持掩码，而不是写入 0 或均值。

## 特征定义

所有特征都变换到**查询/候选帧自身的根局部空间**（平移到根节点、旋转 `-heading`），因此同一动作在不同世界朝向下仍可匹配。特征分三组：

- **pose（姿态，4 维）**：左脚、右脚相对根节点的局部坐标 `(x, y)`。脚部通道缺失时对应维度掩码为 `false`。
- **motion（运动，4 维）**：局部线速度 `(vx, vy)` 与局部加速度 `(ax, ay)`。速度始终参与；加速度不可推导时掩码。
- **trajectory（未来轨迹，默认 9 维）**：在 `horizons`（默认 `0.2s / 0.5s / 1.0s`）三个时刻各取一个采样点，每点为局部偏移 `(x, y)` 与相对朝向差（已 wrap 到 `(-π, π]`）。

轨迹点通过「最近时间匹配」选取：只接受与目标时间相差不超过 `trajectoryTolerance`（默认 0.08s）的点；找不到点则该时刻的 3 个维度全部掩码。非循环片段末尾没有足够未来数据时，对应维度自然掩码。

### 代价计算

对每个组，仅在查询帧与候选帧**双方都存在**的通道上比较：

```
groupCost = mean( ((q_i - c_i) / scale_i)² )   // 只对未掩码通道求均值
totalCost = w_pose * poseCost + w_motion * motionCost + w_trajectory * trajectoryCost
```

- 某组在双方之间没有任何共同通道时，该组代价为 `null`，权重自动不参与，候选结果的 `missingGroups` 会列出该组；不会用 0 或平均值顶替。
- 三组全部不可比时该候选帧被丢弃；没有任何可用候选时返回 `[]`。
- 每个 `scale` 必须为正数、每个 `weight` 必须为非负数，构建期校验，非法配置直接抛错。

### 确定性排序

候选先按 `cost` 升序；代价完全相等时依次按 `clipId` 字典序、`frameIndex` 升序打破平局。排序不依赖 `Map`/对象枚举顺序或浮点随机性，相同输入（无论片段插入顺序如何）返回完全一致的结果。

## 查询选项与候选结构

```js
queryMotionIndex(index, frame, {
  topK: 10,                       // 正整数
  excludedClipIds: ['clip-a'],    // 硬排除整段片段，避免立刻切回刚播过的动作
  maxCost: null,                  // 可选代价上限；超过上限的候选被过滤
});
```

返回：

```js
[{
  clipId, frameIndex, time, loop,
  cost,                  // 总代价
  costs: { pose, motion, trajectory },  // 分组代价，缺失组为 null
  missingGroups: [/* 'pose' | 'motion' | 'trajectory' */],
}]
```

## 循环片段与接缝

- 循环片段（`loop: true`）的轨迹点允许落在片段时长之外，构建期按帧周期取模回绕，因此末帧的未来轨迹指向片段开头，接缝两侧特征连续。
- 提供 `circularFrameDistance(frameIndexA, frameIndexB, frameCount)` 计算循环帧距，供上层播放逻辑判断「跨越接缝的相邻帧」。
- 检索本身不跳帧：跨周期的查询仍映射到周期内最匹配的帧索引，接缝测试验证末帧的下一周期查询命中首帧附近而不是产生空洞。

## 测试数据

`test/synthetic.js` 生成四类合成片段：

- `straight`：恒速直走；
- `turn`：恒定转弯率的弧线行走；
- `stop`：线性减速到零速度的急停（含真实零速度帧）；
- `loop-walk`：沿圆周行走、首尾姿态重合的循环片段。

另有缺失右脚、缺失全部脚部通道、缺失轨迹、单帧片段等变体。`test/motion-index.test.js` 覆盖：相似动作命中、片段排除、权重改变排序、缩放生效、循环接缝帧距、重复查询一致、平局稳定排序、缺失通道掩码、零速度、无候选、配置校验等 20 个用例。

## 已知限制

- 仅 2D（`x/y/heading`）；没有 3D 姿态、关节旋转或上半身通道。
- 暴力线性扫描全部帧，没有 KD-tree / BVH 等空间加速结构，数据量大时需要另加索引层。
- 轨迹是每帧预先烘焙的输入，库不会自动外推或生成未来轨迹。
- 排除规则是「整段片段硬排除 + 可选代价上限」，没有实现片段内冷却时间、最小播放时长或基于根位移的接缝跳变阈值，这些属于上层播放器策略。
- 加速度依赖帧间差分，输入帧率不均时差分结果会反映该不均匀性；建议先统一重采样。
- 特征只做根局部空间归一化，没有骨盆高度、脚步 IK 标签或接触相位等高级通道。
- 分组代价是通道均方差，各通道默认缩放为 1，实际项目通常需要按数据分布重新标定 `scales`。
