# BoW（视觉词袋）库 学习与使用指南

> 面向正在系统熟悉 SLAM 常用库/函数的你。
> 与《视觉SLAM十四讲》第 11 讲（回环检测）、《自动驾驶与机器人中的SLAM技术》回环检测章节配套看效果最好。
>
> 本指南里的每个 API 签名、常量、评分公式都对照过源码（DBoW2 / ORB-SLAM2 的 `KeyFrameDatabase` / DBoW3），
> 不是凭印象写的；标注「⚠️ 未验证」的部分请以你本机为准。

---

## 目录

- [0. 30 秒速览与文件清单](#0-30-秒速览与文件清单)
- [1. BoW 在 SLAM 里到底解决什么问题](#1-bow-在-slam-里到底解决什么问题)
- [2. 原理最小集（够看懂代码即可）](#2-原理最小集够看懂代码即可)
- [3. 库谱系与选择建议](#3-库谱系与选择建议)
- [4. 环境搭建](#4-环境搭建)
- [5. API 详解与速查](#5-api-详解与速查)
- [6. 三条典型工作流](#6-三条典型工作流)
- [7. ORB-SLAM2 是怎么用 BoW 的（源码导读）](#7-orb-slam2-是怎么用-bow-的源码导读)
- [8. 调参与经验值](#8-调参与经验值)
- [9. 性能与内存](#9-性能与内存)
- [10. 常见坑清单](#10-常见坑清单)
- [11. 学习路径：8 个自测实验](#11-学习路径8-个自测实验)
- [12. 延伸阅读](#12-延伸阅读)

---

## 0. 30 秒速览与文件清单

**一句话**：BoW 把一帧图像压成一个稀疏向量（词袋向量），于是「这地方我来过吗」变成一次稀疏向量相似度检索。
它只负责**快速产出候选**，回环是否成立必须靠几何验证（PnP / Sim3 + 位姿图优化）。

**本目录文件**：

| 文件 | 说明 | 状态 |
|---|---|---|
| `BoW库学习与使用指南.md` | 本文件 | — |
| `BoW-API速查表.md` | 只查签名用的精简对照表 | — |
| `examples/bow_from_scratch.cpp` | **纯标准库**的最小 BoW（层次 k-means + TF-IDF + 倒排索引 + L1 评分） | ✅ 已编译运行，输出见 `bow_from_scratch_expected_output.txt` |
| `examples/bow_from_scratch_expected_output.txt` | 上面那个程序的真实运行输出 | ✅ |
| `examples/dbow2_orb_loop_demo.cpp` | DBoW2 真实用法（ORB + ORBvoc.txt + 检索 + 直接索引） | ⚠️ 无真实库可链接；已用桩头文件通过 `-Wall -Wextra` 语法检查，签名逐个对照头文件 |
| `examples/dbow3_train_and_query.cpp` | DBoW3 训练/加载/transform/建库/检索/持久化 | ⚠️ 同上 |
| `examples/dbow3_python_quickstart.py` | Python 两条路线（pyDBoW3 / OpenCV BOW） | ⚠️ 本机无 Python，未运行 |
| `examples/CMakeLists.txt` | 三个示例的构建脚本 | ⚠️ 本机无 cmake，未验证 |
| `.syntaxcheck/` | 只用于语法检查的 OpenCV/DBoW 桩头文件（不是真实库） | — |

**建议阅读顺序**：本文 §2 → 编译跑通 `bow_from_scratch.cpp`（10 分钟，能建立直觉）→ §5/§6 → 有 OpenCV 环境后跑 `dbow2_orb_loop_demo.cpp` → §7 读 ORB-SLAM 源码 → §11 做实验。

```bash
# 立刻能跑的部分（只需要一个 C++ 编译器）
cd examples
g++ -std=c++11 -O2 -o bow_from_scratch bow_from_scratch.cpp
./bow_from_scratch
```

---

## 1. BoW 在 SLAM 里到底解决什么问题

视觉 SLAM 的漂移是原理性的：只靠前端跟踪 + 局部优化，误差会随时间累积，走一圈回到原点，估计的位姿对不上。
唯一能"清零"的手段就是**认出自己来过这里**，然后做全局校正。这就是回环检测（loop closure）。

把问题抽象一下：

| 检测手段 | 原理 | 优点 | 缺点 |
|---|---|---|---|
| 几何近邻 | 当前位姿和历史位姿距离小于阈值 | 简单 | 漂移正是位姿不准的原因，越飘越检测不到 |
| 外观识别（BoW / 学习型描述子） | 图像"看起来像" | 与漂移**解耦**，不依赖位姿精度 | 会误检（perceptual aliasing），需要几何验证兜底 |

所以 SLAM 里的回环检测 = **外观召回（recall 优先）+ 几何确认（precision 兜底）**。BoW 站在前半段。

对 BoW 的工程要求也是这么来的：

1. **快**：每个新关键帧都要查一次全历史，必须是常数级/对数级，不能遍历所有历史帧做特征匹配。
2. **判别力够**：要能区分"相似但不同"的地方（走廊、货架、树），这就要求把"到处都出现的词"权重压下去 —— 这是 TF-IDF 存在的理由。
3. **可扩展**：地图越长库越大，检索耗时不能线性增长 —— 倒排索引 + 层次词典存在的理由。
4. **容忍外观变化**：视角/光照/尺度变化 —— 由底层描述子（ORB 等）承担一部分。
5. **能反查特征**：候选确定后要快速算相对位姿，所以要能回答"候选帧里哪些特征和当前帧落在同一个视觉词上" —— 直接索引存在的理由。

同一个数据库换个查询入口就是**重定位**（跟踪丢失后用当前帧去查库）。区别只在：
回环要求排除共视帧、有 `minScore` 门限、要求时间一致性；重定位不排除共视（因为已经跟丢了），门限也更宽松。

> 记住这条分工：**BoW 只产出候选（candidate），位姿求解与优化才决定它是不是回环。**
> 读 ORB-SLAM 时你会在 `LoopClosing` 里看到：`DetectLoopCandidates` → 特征匹配 → Sim3 求解 → `Optimizer::OptimizeEssentialGraph`，
> 后面三步才是"判定"。

---

## 2. 原理最小集（够看懂代码即可）

### 2.1 视觉词典 = 一棵层次 k-means 树

不是"聚类成 k 个词"，而是**递归地在每一层聚成 k 类**：

```
根
├── 子节点1（第1层，k个分支）
│   ├── 子节点1-1（第2层）
│   │   └── ... 一直到第 L 层
│   └── ...
└── ...
```

- `k` = 分支因子（branching factor），`L` = 深度（depth levels）。
- **叶子节点 = 词（word）**，词的总数最多 `k^L`。ORB-SLAM 的词典 `k=10, L=6` → 最多 100 万个词。
- 为什么用树而不用扁平 k-means：扁平 k-means 找最近词是 O(词数)=O(10^6)；
  树上是每层比 k 次、共 L 层，O(L·k)=O(60)，快 4 个数量级。这是 BoW 能做到实时的关键。
- **二进制描述子的"均值"不能用算术平均**。DBoW2 的 `FORB::meanValue()` 是**逐 bit 多数投票**：
  某一 bit 在簇内过半为 1 就取 1。这也意味着 BoW 天然适配二进制特征。
- 初始化用 **kmeans++**（`initiateClustersKMpp`），避免随机初始化导致的坏簇。

### 2.2 从特征到向量：`transform()`

对一帧的每个描述子：**从根开始，逐层贪心选最近的子节点，直到叶子**。落到的叶子的 `word_id` 就是这个词。

```
描述子 d → 根 → 选最近的子节点 → ... → 叶子 = word_id
```

然后累计权重，得到 **BowVector** = `std::map<WordId, WordValue>`（稀疏、按 word_id 有序 —— 有序很关键，见 §2.3）。

**权重方案（`WeightingType`）**：

| 枚举 | 值 | 含义 |
|---|---|---|
| `TF_IDF` | 0 | 词频 × 逆文档频率（SLAM 默认） |
| `TF` | 1 | 只用词频 |
| `IDF` | 2 | 只用逆文档频率（同一帧内重复出现只算一次） |
| `BINARY` | 3 | 0/1 |

TF-IDF 的 idf 部分在**训练词典时**算好，写进每个词的 `weight`：

```
idf(word) = ln( N / Ni )
    N  = 训练图像总数
    Ni = 包含该词的训练图像数
```

直觉：**到处都出现的词（Ni 大）→ idf 小 → 权重低**；罕见词权重高。这就是"把走廊、天空这类没判别力的词压下去"。
若某词在训练集里只出现 1 次，`idf = ln(N)` 会极大 —— 所以**词典不能训得比训练数据还"细"**（§11 实验 4 会亲手看到这个现象）。

`transform()` 里对 `TF_IDF` 的处理是：每命中一次就 `addWeight(id, idf)`（命中次数即 tf），
如果评分方式需要归一化（L1/L2 需要），最后做一次向量归一化。所以 **BowVector 通常是归一化过的**。

**两种索引**（回环检测两个阶段各用一个）：

| 索引 | 结构 | 方向 | 用途 |
|---|---|---|---|
| 倒排索引 inverted file | `word_id → [(entry_id, weight)]` | 词 → 帧 | 检索：只比对共享词的帧，而不是全库 |
| 直接索引 direct file | `entry_id → {node_id → [feature_idx]}` | 帧 → 特征 | 几何验证：候选帧只需匹配共享节点上的特征 |

`transform(features, v, fv, levelsup)` 里那个 **`levelsup`** 就是控制直接索引"往上数几层"：
`levelsup=0` 用叶子（最细），ORB-SLAM 用 **`levelsup=4`**（粗一些，让两帧有更多机会落到同一节点上，提高匹配召回）。

### 2.3 相似度：评分方式与**取值方向**（最容易搞错的地方）

DBoW2/DBoW3 的评分函数实现（`ScoringObject.cpp`，已核对源码）：

| `ScoringType` | 值 | 公式 | 范围 | 方向 |
|---|---|---|---|---|
| `L1_NORM` | 0 | `1 - 0.5·‖v-w‖₁`（等价于 `-½·Σ(|vᵢ-wᵢ|-|vᵢ|-|wᵢ|)`） | [0,1] | **越大越好** |
| `L2_NORM` | 1 | `1 - sqrt(1 - Σvᵢwᵢ)` | [0,1] | **越大越好** |
| `CHI_SQUARE` | 2 | `2·Σ vᵢwᵢ/(vᵢ+wᵢ)` | [0,1] | **越大越好**（名字叫 distance，其实是相似度） |
| `KL` | 3 | `Σ vᵢ·ln(vᵢ/wᵢ)` | 不可缩放 | **越小越好**（唯一的例外！） |
| `BHATTACHARYYA` | 4 | `Σ sqrt(vᵢwᵢ)` | [0,1] | **越大越好** |
| `DOT_PRODUCT` | 5 | `Σ vᵢwᵢ` | 不可缩放 | **越大越好** |

> `ORBvoc.txt` 用的是 **L1_NORM (0) + TF_IDF (0)**，所以 `score ∈ [0,1]`，1 = 完全相同。
> 这意味着 ORB-SLAM 里 `minScore` 这类阈值也是 [0,1] 量纲的。

**好消息**：`TemplatedDatabase::query()` 内部会按评分方式自动选排序方向，
所以结果数组**永远是"越靠前越好"**。**坏消息**：KL 情况下 `Score` 数值越小越好，如果你自己写检索/写阈值判断，一定要区分方向。

DBoW2 还有个容易踩的魔数：`TemplatedDatabase.h` 里的 `static int MIN_COMMON_WORDS = 5;`
—— **公共词少于 5 个的候选直接被丢掉**（只作用于 `CHI_SQUARE` 和 `BHATTACHARYYA` 两条查询路径，见源码 `queryChiSquare`/`queryBhattacharyya`）。
所以用这两种评分时，`QueryResults` 可能比预期少很多。

### 2.4 参数一览（就是构造函数的四个参数）

```cpp
OrbVocabulary voc(k, L, weightingType, scoringType);
// 典型: k = 10, L = 5~6, TF_IDF, L1_NORM
```

| 参数 | 典型值 | 变大意味着 |
|---|---|---|
| k 分支 | 10 | 词数 k^L 增长更快、单层比较变慢、训练更慢 |
| L 深度 | 5~6 | 词数指数增长、判别力变强、内存与加载时间暴涨 |
| 词数 | 10^5 ~ 10^6 | 判别力↑，但需要更多训练数据支撑，内存↑ |

---

## 3. 库谱系与选择建议

同一套算法的几个分支，**文档和 API 各不相同，混用必踩坑**：

| 库 | 形态 | 词典格式 | 描述子 | 特点 |
|---|---|---|---|---|
| **DBoW2**（`dorian3d/DBoW2`，原版） | 模板类 `TemplatedVocabulary<TDescriptor,F>` | 只有 `cv::FileStorage`（`.yml.gz`） | ORB/BRIEF/SURF 通过 `FORB`/`FBrief`/`FSurf64` 策略类 | 最"学术"，代码最干净，回环检测的开山之作 |
| **DBoW2 魔改版**（ORB-SLAM2/3 的 `Thirdparty/DBoW2`） | 同上 + 文本词典读写 | 多了 **纯文本 `ORBvoc.txt`** | 同上 | **加了 `loadFromTextFile()` / `saveToTextFile()`**，ORB-SLAM 全靠它加载那 145MB 的词典 |
| **DBoW3**（`rmsalinas/DBow3`） | **非模板**：`DBoW3::Vocabulary` / `Database` | 二进制 `.dbow3` / 文本 / YAML，`load()` 自动识别 | 固定 `cv::Mat` | 更好用：加载快、`transform` 有直接吃矩阵的重载、官方给现成 `orbvoc.dbow3` |
| **pyDBoW3**（`foxis/pyDBoW3`） | Python 绑定（Boost.Python） | 同 DBoW3 | 同 DBoW3 | 最后一次发布是 0.2(2019)，只有 cp37 wheel；新版 Python 需自行编译 |
| **pyslam 的 pydbow3** | Python 绑定 | 同 DBoW3 | 同 DBoW3 | 维护得更新，从源码构建 |
| **学习型方案**（NetVLAD / AnyLoc + FAISS） | 模型 + 向量检索库 | 神经网络权重 | 单张图 → 一个稠密描述子 | 跨季节/光照/大视角更鲁棒，是当前研究主流；但引入模型依赖与 GPU |

**给"熟悉库和函数"阶段的建议**：

1. **学原理**：读原版 DBoW2 的 `TemplatedVocabulary.h`（一个文件就能读懂全部算法）+ 本目录的 `bow_from_scratch.cpp` 对照。
2. **读工程**：读 ORB-SLAM2 的 `Thirdparty/DBoW2`（看文本词典读写）+ `src/KeyFrameDatabase.cc`（看候选生成的真实工程写法）。
3. **自己写实验/小工具**：用 **DBoW3**，API 最省事，二进制词典加载快。
4. **知道边界**：BoW + 手工特征在强外观变化下会失效；往深了走就是学习型全局描述子，检索侧换成 FAISS/HNSW 而不是视觉词典树。思路是同一条：全局描述子 + 近似最近邻 + 几何验证。

---

## 4. 环境搭建

### 4.1 拿到词典文件（这是最容易被卡住的一步）

词典必须**与你的描述子类型一致**，不能混用：

| 词典 | 来源 | 大小（约） | 适用 |
|---|---|---|---|
| `ORBvoc.txt` | ORB-SLAM2 仓库 `Vocabulary/ORBvoc.txt.tar.gz`（ORB-SLAM3 同） | 压缩 ~43MB / 解压 ~145MB | ORB 描述子 + DBoW2 魔改版 / DBoW3 |
| `orbvoc.dbow3` | DBoW3 仓库提供的二进制格式 | ~43MB | ORB 描述子 + DBoW3（加载快得多） |
| 自己训练的 `voc_trained.yml.gz` / `.dbow3` | §6 工作流 B | 看你训练规模 | 特定场景、特定描述子 |

```bash
# ORB-SLAM2 的词典
wget https://github.com/raulmur/ORB_SLAM2/raw/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xzvf ORBvoc.txt.tar.gz          # 得到 ORBvoc.txt
head -1 ORBvoc.txt                   # 应该是: 10 6 0 0   （k=10, L=6, scoringType=0, weightingType=0）
wc -l ORBvoc.txt                     # 约 111 万行 = 节点总数 (k^(L+1)-1)/(k-1) 减掉根节点
```

> `head -1` 这一行别跳过：`10 6 0 0` 的含义是 `k L scoringType weightingType`。
> 注意顺序 —— **先 scoring 后 weighting**，和构造函数 `Vocabulary(k, L, weighting, scoring)` 的参数顺序是**反的**。

### 4.2 编译 DBoW3（推荐用它的场景）

```bash
git clone https://github.com/rmsalinas/DBow3.git
cd DBow3 && mkdir build && cd build
cmake .. && make -j4 && sudo make install
# 默认装到 /usr/local，产出 libDBoW3.a / libDBoW3.so
```

### 4.3 不用装库：直接借用 ORB-SLAM2 的源码树编译（最省事）

```bash
g++ -std=c++11 -O2 examples/dbow2_orb_loop_demo.cpp -o dbow2_demo \
    $(pkg-config --cflags --libs opencv4) \
    -IORB_SLAM2/Thirdparty/DBoW2 \
    ORB_SLAM2/Thirdparty/DBoW2/src/*.cpp \
    ORB_SLAM2/Thirdparty/DBoW2/DUtils/*.cc
```

或者用本目录的 CMake：

```bash
cd examples && mkdir build && cd build
cmake -DORBSLAM_ROOT=/path/to/ORB_SLAM2 ..        # 借用 ORB-SLAM 源码树里的 DBoW2
cmake -DDBOW3_INCLUDE_DIR=/usr/local/include -DDBOW3_LIB=/usr/local/lib/libDBoW3.so ..  # 用装好的 DBoW3
make -j4
```

### 4.4 Windows / 本机情况说明

- 本机现状：有 MinGW g++ 8.1.0，**没有** cmake / OpenCV / Python，所以只有纯标准库的 `bow_from_scratch.cpp` 被实际编译运行验证过。
- Windows 上装 OpenCV 后用 `-DOpenCV_DIR=...` 或 CMake 的 `find_package(OpenCV)`；MSVC 编译 DBoW2 注意 `Multiple definition` / `std::min` 之类的小问题。
- **内存要吃得住**：加载 100 万词的 ORBvoc 常驻内存数百 MB，训练时更高。32 位进程基本没戏。

---

## 5. API 详解与速查

完整签名对照见 `BoW-API速查表.md`，这里只讲"怎么用 + 坑在哪"。

### 5.1 DBoW2 的绑定方式：模板 + 描述子策略类

DBoW2 是模板类，你要把"描述子类型"和"描述子操作"绑进去。ORB 的绑定就是 ORB-SLAM2 `include/ORBVocabulary.h` 的全部内容：

```cpp
#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2 {
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
}
```

`FORB` 提供的接口（`FORB.h` 已核对）：

| 成员 | ORB 的值/含义 |
|---|---|
| `FORB::TDescriptor` | `cv::Mat`，必须是 **CV_8U** |
| `FORB::L` | `32`（字节数；文本词典一行的 32 个数字就是一个描述子） |
| `FORB::distance(a,b)` | 汉明距离 |
| `FORB::meanValue(...)` | 逐 bit 多数投票 |
| `FORB::toString / fromString` | 词典文本格式的序列化 |

### 5.2 DBoW2 词典：常用成员

```cpp
OrbVocabulary voc;                 // 或者一步到位 voc(k, L, TF_IDF, L1_NORM)
voc.loadFromTextFile("ORBvoc.txt"); // ⚠️ 只有 ORB-SLAM 的魔改版有；原版用 voc.load("voc.yml.gz")
bool bad = voc.empty();             // 一定先查！空词典会静默返回空结果
std::cout << voc;                   // 打印 k/L/weighting/scoring/词数
voc.size();                         // 词数
voc.getBranchingFactor(); voc.getDepthLevels(); voc.getEffectiveLevels();
voc.getWord(wid); voc.getWordWeight(wid);
voc.getParentNode(wid, levelsup);   // 往上数 levelsup 层的节点 id（直接索引用）
voc.getWordsFromNode(nid, words);   // 某节点下所有词
voc.stopWords(0.001);               // 把 weight < 阈值的词停用（weight 置 0），返回停用个数
voc.score(bow1, bow2);              // 成对评分，ORB-SLAM 的候选过滤就用这个
```

```cpp
// 三种 transform 用法
voc.transform(vDesc, bow);                    // 只要 BowVector
voc.transform(vDesc, bow, feat, levelsup);    // BowVector + FeatureVector（回环检测用这个）
WordId id = voc.transform(oneDescriptor);     // 单个描述子 -> 词
```

### 5.3 DBoW2 数据库

```cpp
OrbDatabase db(voc, /*use_di=*/false, /*di_levels=*/0);  // 注意：会拷贝一份词典！
db.allocate(num_entries, 0);        // 预分配倒排索引，避免扩容
EntryId id = db.add(bow);                        // 或用 add(features, &bow, &fvec)
DBoW2::QueryResults ret;
db.query(bow, ret, /*max_results=*/5, /*max_id=*/-1);
for (auto& r : ret) printf("%u %.4f\n", r.Id, r.Score);

const DBoW2::FeatureVector& fv = db.retrieveFeatures(id);  // 直接索引（需 use_di=true）
db.save("db.yml.gz");  db.load("db.yml.gz");      // 词典也一起存
```

**`Result` 结构**（DBoW2/DBoW3 相同）：`Id`（entry id）、`Score`（相似度）、
`nWords`（公共词数，**只在 CHI_SQUARE / BHATTACHARYYA 下才被填充**）、以及 `chiScore`/`bhatScore` 等调试字段。

**`max_id` 的坑**：文档注释写的是"只返回 `id <= max_id`"，但源码里是 **`entry_id < max_id`（严格小于）**。
两种版本都是这个实现，按"上界开区间"理解。

### 5.4 DBoW3 的差异（写代码时最容易转不过弯的地方）

```cpp
#include "DBoW3/DBoW3.h"        // 不再是模板，没有 typedef
DBoW3::Vocabulary voc(10, 6, DBoW3::TF_IDF, DBoW3::L1_NORM);
voc.create(trainingFeatures);   // vector<vector<cv::Mat>>（每张图一组）或 vector<cv::Mat>（每个元素一个描述子）
voc.save("voc.dbow3");          // .yml/.yaml -> OpenCV YAML；其他后缀 -> 压缩二进制
voc.load("orbvoc.dbow3");       // 自动识别二进制/文本/YAML；失败时 throw

DBoW3::BowVector bow;  DBoW3::FeatureVector feat;
voc.transform(vDesc, bow, feat, 4);   // 和 DBoW2 一致的写法
voc.transform(descMat, bow);          // ✨ DBoW3 独有：直接吃 Nx32 的 CV_8U 矩阵，不用按行拆

DBoW3::Database db(voc, /*use_di=*/true, /*di_levels=*/4);
db.add(bow, feat);
DBoW3::QueryResults ret;
db.query(bow, ret, 5, -1);
const DBoW3::FeatureVector& fv = db.retrieveFeatures(id);
db.save("kf_db.yml.gz");
```

| 差异点 | DBoW2 | DBoW3 |
|---|---|---|
| 类形态 | `TemplatedVocabulary<TDescriptor,F>` | `Vocabulary`（固定 `cv::Mat`） |
| 描述子拆分 | 必须 `vector<cv::Mat>`（每行一个） | 两种都行，`transform(Mat, bow)` 直接吃矩阵 |
| 文本词典 | 原版**没有**；魔改版有 `loadFromTextFile` | `load()` 自动识别 |
| 二进制词典 | 无 | `.dbow3`，加载快、体积小 |
| 抛异常风格 | `throw std::string(...)` | 混用 `std::string` / 异常 |

### 5.5 枚举数值表（因为文本词典首行是数字）

```cpp
// WeightingType            // ScoringType
TF_IDF = 0                  L1_NORM = 0
TF     = 1                  L2_NORM = 1
IDF    = 2                  CHI_SQUARE = 2
BINARY = 3                  KL = 3
                            BHATTACHARYYA = 4
                            DOT_PRODUCT = 5
```

`ORBvoc.txt` 首行 `10 6 0 0` ⇒ `k=10, L=6, scoringType=L1_NORM, weightingType=TF_IDF`。
魔改版的 `loadFromTextFile` 会做合法性校验：`k∈[0,20], L∈[1,10], scoring∈[0,5], weighting∈[0,3]`，
不符合就打印 "This is not a correct text file!" 并返回 `false`。

---

## 6. 三条典型工作流

### 工作流 A：用现成词典做检索（最常用）

```cpp
OrbVocabulary voc;
voc.loadFromTextFile("ORBvoc.txt");            // 1) 加载词典（秒级）
OrbDatabase db(voc, false, 0);                 // 2) 建库
for (每个历史关键帧 kf) {
  voc.transform(kf.vDesc, kf.bow, kf.feat, 4); // 3) 提 BowVector（每帧一次）
  db.add(kf.bow);                              // 4) 入库
}
DBoW2::QueryResults ret;
db.query(curBow, ret, 5, -1);                  // 5) 查询 → 候选
// 6) 几何验证：对候选做特征匹配 + Sim3/PnP → 才是回环
```

完整可运行版本：`examples/dbow2_orb_loop_demo.cpp`、`examples/dbow3_train_and_query.cpp`。

### 工作流 B：自己训练词典

```cpp
// 1) 准备训练集：几百到几万张图像，覆盖你关心的场景
std::vector<std::vector<cv::Mat> > training;   // 每张图的描述子（CV_8U, 32 列）
// 2) 训练
OrbVocabulary voc(10, 5, DBoW2::TF_IDF, DBoW2::L1_NORM);
voc.create(training);
// 3) 保存
voc.saveToTextFile("myvoc.txt");               // 魔改版；原版用 voc.save("myvoc.yml.gz")
```

要点（`create()` 内部会做 `getFeatures → HKmeansStep 递归建树 → createWords → setNodeWeights`）：

- **词数必须远小于训练特征总数**。我本机跑的玩具实验：7200 个训练特征训 `k=10, L=4`，
  结果生成 **6448 个词**（几乎一词一特征），于是 idf 全部趋近 `ln(60)=4.0943`，
  TF-IDF 退化成"近似精确匹配"，判别力几乎为零。真实词典用海量数据训练，就是为了让每个词在训练集里被**多张图**看到。
- 经验：**训练特征总数 / 词数 ≳ 100** 才比较健康（即 10 万词至少 1000 万级训练特征）。
- kmeans++ 有随机性（`DUtils::Random`），同数据两次训练结果不一定完全相同；要复现就固定随机种子。
- 词典是**场景相关**的：拿室内数据训的词典去跑室外会掉点，反之亦然。

### 工作流 C：在 ORB-SLAM2 上做实验

| 想改什么 | 改哪里 |
|---|---|
| 换词典 / 调词典参数 | `System.cc` 里 `mpORBVocabulary` 的加载；或直接换 `ORBvoc.txt` |
| 看每帧 BoW | `Frame::ComputeBoW()` 后打印 `mBowVec.size()` |
| 调回环灵敏度 | `LoopClosing` 传给 `DetectLoopCandidates` 的 `minScore` |
| 调候选生成策略 | `KeyFrameDatabase::DetectLoopCandidates` 里的 `0.8f` / `0.75f` 常量、`GetBestCovisibilityKeyFrames(10)` |
| 换评分方式 | 词典构造时的 `ScoringType`（同时要注意方向，见 §2.3） |
| 排除"太新的帧" | 参考重定位路径，或自己加 id 上界（DBoW2 的 `max_id`） |

---

## 7. ORB-SLAM2 是怎么用 BoW 的（源码导读）

这一节的值在于：**它没用 DBoW2 的 `TemplatedDatabase`，而是自己写了一个 `KeyFrameDatabase`**。理解为什么，你就理解了 BoW 库的边界。

### 7.1 数据流

```
Frame::ComputeBoW()
  └─ Converter::toDescriptorVector(mDescriptors)   // Nx32 -> vector<cv::Mat>
  └─ mpORBVocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4)
        → 每个 KeyFrame 身上从此常驻 mBowVec / mFeatVec

KeyFrameDatabase::add(pKF)
  └─ mvInvertedFile.resize(voc.size())             // vector<list<KeyFrame*>>
  └─ for (word, w) in pKF->mBowVec: mvInvertedFile[word].push_back(pKF)
        → 注意：存的是 KeyFrame* 指针，不是 entry id
```

### 7.2 `DetectLoopCandidates(pKF, minScore)` 的六个步骤（已对照源码）

1. **取共视集合**：`spConnectedKeyFrames = pKF->GetConnectedKeyFrames()`。
2. **扫倒排索引收集"共享词的帧"**，并**跳过共视帧**（`if(!spConnectedKeyFrames.count(pKFi))`）。
   同时用 `pKFi->mnLoopWords++` 统计**公共词个数**（用 `mnLoopQuery` 标记避免重复计数）。
   —— 这一步就是 DBoW2 `query` 里倒排索引的角色，但多了一句"排除共视"。
3. **自适应公共词门限**：`minCommonWords = maxCommonWords * 0.8f`，
   只保留 `mnLoopWords > minCommonWords` 的帧。比写死一个常数稳健得多。
4. **算分并卡门限**：`float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);`
   保留 `si >= minScore` 的候选。注意这里用的是**词典的成对评分**，不是 `Database::query`。
5. **按共视累加分数**：对每个候选，取它共视最强的 10 帧（`GetBestCovisibilityKeyFrames(10)`），
   把它们中"也满足公共词门限"的分加到 `accScore` 上，并记录分数最高的那帧。
   —— 单一的分数容易被噪声/误检骗，**一簇帧同时高分**才可信。
6. **相对门限筛选**：`minScoreToRetain = 0.75f * bestAccScore`，返回所有 `accScore` 高于它的帧。

### 7.3 `DetectRelocalizationCandidates(F)` 的差异

- 入参是 `Frame*` 而不是 `KeyFrame*`（跟踪已丢，没有共视图）。
- **不排除共视帧**（无共视可依），公共词门限同样是 `0.8f * maxCommonWords`。
- **没有 `minScore` 门限**，最终门限是 `0.75f * bestAccScore`（`bestAccScore` 从 0 起算）。
- 判定阶段用 PnP 求相对位姿，再 `Optimizer::OptimizeEssentialGraph`。

### 7.4 为什么不用 DBoW2 的 `TemplatedDatabase`？

因为工程需求超出了它的模型：

| 需求 | DBoW2 `TemplatedDatabase` | 结果 |
|---|---|---|
| 存 `KeyFrame*`（要访问位姿、特征、共视图） | 只存 `EntryId` | 不够用 |
| 查询时排除当前帧的共视帧 | 只支持 `max_id`（按 id 上界） | 不够用 |
| 自适应公共词门限（0.8×max） | 固定 `MIN_COMMON_WORDS = 5` | 不够用 |
| 按共视簇累加分数 | 只有单帧分数 | 不够用 |
| 内存里避免词典副本 | `Database` 拷贝整份词典 | 浪费几百 MB |

所以 ORB-SLAM 只借用词典（`mpVoc->score` / `transform`），自己实现了倒排索引和候选逻辑。
**这也是你写自己的回环模块时的推荐架构**：词典用库（加载/transform/score），倒排和候选策略自己写（20 行）。

---

## 8. 调参与经验值

| 参数 | 建议 | 说明 |
|---|---|---|
| `k` | 10 | 常见固定值。改它收益小、影响大 |
| `L` | 5~6（对应 10^5~10^6 词） | 词数不足→判别力差；词数过剩→需要天量训练数据，否则 idf 失衡 |
| 权重/评分 | `TF_IDF` + `L1_NORM` | ORB-SLAM 的配置，生态里最成熟，阈值经验最多 |
| `levelsup` | 4 | ORB-SLAM 的选择；越小越精细（匹配点少），越大越粗（召回高、误匹配多） |
| `minScore` | 必须**在自己的数据上标定** | 不同词典/场景不可移植。做法：人工标出真回环，画 score 的 PR 曲线取工作点 |
| 公共词门限 | `0.8 * maxCommonWords` | ORB-SLAM 的做法，比常数稳健 |
| 一致性要求 | 连续 ≥3 个候选指向同一地点 / 共视簇累加 | 抑制偶发误检的关键 |
| `stopWords(minWeight)` | 谨慎用 | 会**永久**把词的 weight 置 0（再用更小阈值调用也无法恢复） |

**标定 minScore 的正确姿势**（别抄别人的数字）：

1. 用你手上的数据集跑一遍，人工标出真回环帧对（GT）。
2. 记录每次查询的 top-1/top-k 分数，画 PR 曲线。
3. 取"召回率够用、误检率可接受"的那个阈值。
4. 记住：**误检比漏检更贵**（错误回环会毁掉整条轨迹），所以工作点通常偏保守，靠几何验证补召回。

---

## 9. 性能与内存

| 项目 | 量级 | 备注 |
|---|---|---|
| `ORBvoc.txt` 加载 | 数秒 | 文本格式解析 100 万行；二进制 `.dbow3` 快数倍 |
| 词典常驻内存 | 数百 MB（100 万词） | 多实例共享同一份，别拷贝 |
| 单帧 `transform`（1000 特征） | 数毫秒 ~ 十几毫秒 | 与词数/深度正相关 |
| 玩具实测（本机） | 120 特征 + 6448 词 → **约 1.0 ms/帧**（多次运行 1.00~1.08 ms） | 见 `bow_from_scratch_expected_output.txt` |
| 训练（本机玩具） | 7200 特征 + `k=10,L=4` → **约 1 s** | 真实规模是小时级 |
| `db.query` | 亚毫秒 ~ 毫秒 | 倒排索引，与库大小近似无关 |

工程建议：

- **一份词典，多线程共享**。`transform()` 是 `const`，可并发调用；`Database::add/query` 不是，需自己加锁（ORB-SLAM2 的 `KeyFrameDatabase` 用了 `mMutex`）。
- **别让 `TemplatedDatabase` 拷贝词典**（构造时会把整份词典拷进去，内存直接翻倍）。要么用指针/引用自己包一层，要么用 DBoW3 + 自己写倒排。
- 用 `allocate()` 预分配倒排索引。
- 落盘用二进制格式（`.dbow3` / `.yml.gz` 而非纯文本）。

---

## 10. 常见坑清单

1. **描述子类型/尺寸不匹配**。DBoW2 的 `FORB` 要求 **CV_8U**；有人从别处拿到 CV_32F 的"ORB 描述子"直接喂进去 → 结果全错或崩溃。
   自检：`desc.type()==CV_8U && desc.cols==32`。
2. **忘了把 Nx32 矩阵拆成 `vector<cv::Mat>`**。DBoW2 的 `transform` 要的是"每个描述子一个 `cv::Mat`"，
   直接传整个矩阵编译不过；用 ORB-SLAM 的 `Converter::toDescriptorVector` 等价实现（`descriptors.row(j)`）。
   DBoW3 有吃矩阵的重载，容易和 DBoW2 记混。
3. **词典与描述子类型不匹配**。ORB 词典喂 SIFT/SUPERPOINT/LIGHTGLUE 描述子 → 要么长度不符报错，要么静默乱码。
   换描述子必须重训词典。
4. **`loadFromTextFile` 不存在**。那是 ORB-SLAM 补丁加的方法；原版 DBoW2 只有 `load()`（`cv::FileStorage`，`.yml.gz`）。
   报 "no member named 'loadFromTextFile'" 就是这个原因。
5. **词典首行顺序记反**。`10 6 0 0` 是 `k L scoringType weightingType`，
   而构造函数是 `(k, L, weighting, scoring)`。改评分方式时特别容易搞反。
6. **空词典静默失败**。`empty()` 为真时 `transform()` 直接返回空 BowVector，不报错、不抛异常。
   一定要 `if (voc.empty()) return -1;`。
7. **不排除当前帧 → 自匹配霸榜**。库里有当前帧时，`Score=1.0` 的它自己永远排第一。
   用 `max_id`（注意严格小于）或像 ORB-SLAM 那样排除共视帧。
   本机玩具实验里这一点看得很清楚：包含全部帧时 top-1 是查询帧自己（1.0000），
   排除后 top-1 才变成正确的回环历史帧（0.4370）。
8. **评分方向搞反**。除 KL 外都是越大越好；KL 越小越好。自己写检索排序时必须区分。
9. **拿 `nWords` 当通用判据**。它只在 `CHI_SQUARE`/`BHATTACHARYYA` 路径下被填充。
10. **`MIN_COMMON_WORDS = 5` 静默丢候选**。用这两种评分时公共词少于 5 的候选不会返回。
11. **`Database` 拷贝词典导致内存翻倍**（DBoW2 模板版尤其明显）。
12. **`stopWords()` 不可逆**：weight 被置 0 后，再用更小的阈值调用也恢复不了，只能重新 `create` 或重新加载。
13. **词典是场景相关的**。室内/室外、城市/野外混用会掉召回 —— 这不一定是代码问题。
14. **训练数据量不足导致 idf 失衡**（见 §6 工作流 B 的实测数字）：`idf = ln(N/Ni)`，Ni=1 时权重最大，
   结果少数"只出现过一次"的词垄断了整个向量。

---

## 11. 学习路径：8 个自测实验

每个实验都给了**判据**：跑出来符合预期才算真懂。实验 0 现在就能做，其余需要 OpenCV/DBoW 环境。

**实验 0（立刻可做）**：跑通最小实现，理解数据流
```bash
cd examples && g++ -std=c++11 -O2 -o bow_from_scratch bow_from_scratch.cpp && ./bow_from_scratch
```
判据：能解释输出里"同地点 0.437 / 不同地点 0.009"的差距来自哪里；
能说清 `transform()` 里哪一步产生了 `BowVector` 的稀疏性。

**实验 1**：加载 ORBvoc.txt，打印词典信息与单帧耗时
判据：`voc.size()` 接近 10^6；`voc.getDepthLevels()==6`；transform 耗时在毫秒级。

**实验 2**：自建相似度矩阵（`voc.score`）
判据：同一地点不同视角的分数明显高于不同地点；能说清为什么对角线是 1.0。

**实验 3**：自匹配与 `max_id`
判据：不排除当前帧时 top-1 是自己（≈1.0）；用 `max_id=当前id` 后 top-1 变成正确的历史帧。

**实验 4**：训练自己的词典（200 张图, `k=10,L=5`），与 ORBvoc 对比
判据：能观察到小词典词数不足 / 一词一特征导致的 idf 失衡（打印 `getWordWeight` 分布即可）；
能用同一批查询对比两个词典的召回差异。

**实验 5**：`stopWords(0.001)` 前后对比
判据：BowVector 非零词数下降；能说明它压掉的是"到处都出现的词"还是"罕见词"。

**实验 6**：`levelsup` 取 0/2/4 对直接索引的影响
判据：`FeatureVector` 的节点数随 levelsup 增大而减少（节点更粗）；
能说明这如何影响后续几何验证阶段的匹配召回与耗时。

**实验 7**：给 ORB-SLAM2 加日志
在 `DetectLoopCandidates` 里打印 `maxCommonWords` / `minCommonWords` / 每个候选的 `si` / `bestAccScore`。
判据：能在真回环发生的时刻看到"一簇帧同时高分"；能说明 `0.75f` 这个相对门限在做什么。

**实验 8**：换评分方式
把词典的 `ScoringType` 换成 `DOT_PRODUCT` / `CHI_SQUARE`（需重新加载或重训），比较候选排序。
判据：能解释为什么 `CHI_SQUARE` 下 `nWords` 才有值；能说清 KL 的排序方向差异。

---

## 12. 延伸阅读

**源码（强烈建议直接读，比任何博客都准）**

- DBoW2 原版：[dorian3d/DBoW2](https://github.com/dorian3d/DBoW2) —
  重点是 `include/DBoW2/TemplatedVocabulary.h`、`TemplatedDatabase.h`、`src/ScoringObject.cpp`、`include/DBoW2/FORB.h`
- DBoW3：[rmsalinas/DBow3](https://github.com/rmsalinas/DBow3) — `src/Vocabulary.h`、`src/Database.h`、`src/QueryResults.h`
- ORB-SLAM2：[raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) —
  `include/ORBVocabulary.h`、`Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h`（看 `loadFromTextFile`）、
  `src/KeyFrameDatabase.cc`（候选生成）、`src/LoopClosing.cc`（判定与优化）
- 十四讲配套代码：[gaoxiang12/slambook2](https://github.com/gaoxiang12/slambook2) 的 `ch11`
  （`feature_training.cpp` / `loop_closure.cpp` / `gen_vocab_large.cpp`，用的就是 DBoW3）
- pyDBoW3：[foxis/pyDBoW3](https://github.com/foxis/pyDBoW3)；pyslam 的维护版：[luigifreda/pyslam](https://github.com/luigifreda/pyslam)

**论文**

- Gálvez-López & Tardós, *Bags of Binary Words for Fast Place Recognition in Image Sequences*, T-RO 2012 —— DBoW2 的原始论文，讲清了二进制描述子 + 层次词典 + 直接/倒排索引的完整设计。
- Nistér & Stewénius, *Scalable Recognition with a Vocabulary Tree*, CVPR 2006 —— 层次词典树的源头，L1/L2 评分公式出自这里。

**往下一站**

- 回环的后半段：Sim3 / PnP 求解 + 位姿图优化（ORB-SLAM 的 `Optimizer::OptimizeEssentialGraph`）。
- 学习型全局描述子：NetVLAD、AnyLoc；检索侧换成 FAISS/HNSW。
