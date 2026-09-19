# BoW API 速查表

> 签名逐个对照过源码。`TDescriptor` 在 ORB 场景下 = `cv::Mat`(CV_8U, 1×32)。
> 约定：`using namespace DBoW2;`（DBoW3 则把 `OrbVocabulary` 换成 `DBoW3::Vocabulary`）。

---

## 1. DBoW2 `TemplatedVocabulary<TDescriptor, F>`

```cpp
// ---- 构造 / 创建 ----
TemplatedVocabulary(int k = 10, int L = 5,
                    WeightingType weighting = TF_IDF,
                    ScoringType   scoring   = L1_NORM);
TemplatedVocabulary(const std::string &filename);        // 构造即 load()
TemplatedVocabulary(const char *filename);               // ⚠️ 走 cv::FileStorage，不是文本词典

void create(const std::vector<std::vector<TDescriptor> > &training_features);
void create(const std::vector<std::vector<TDescriptor> > &training_features, int k, int L);
void create(const std::vector<std::vector<TDescriptor> > &training_features,
            int k, int L, WeightingType weighting, ScoringType scoring);

// ---- 查询信息 ----
unsigned int size() const;                 // 词数
bool empty() const;                        // ⚠️ 一定先查！空词典 transform 静默返回空结果
int  getBranchingFactor() const;           // k
int  getDepthLevels() const;               // L
float getEffectiveLevels() const;          // 叶子平均真实深度
TDescriptor getWord(WordId wid) const;     // 词的聚类中心描述子
WordValue   getWordWeight(WordId wid) const;  // 词的 idf
WeightingType getWeightingType() const;
ScoringType   getScoringType() const;
void setWeightingType(WeightingType type);
void setScoringType(ScoringType type);     // 会重建 scoring object

// ---- 变换：描述子 -> 向量 ----
void transform(const std::vector<TDescriptor>& features, BowVector &v) const;
void transform(const std::vector<TDescriptor>& features,
               BowVector &v, FeatureVector &fv, int levelsup) const;  // 回环检测用这个
WordId transform(const TDescriptor& feature) const;                    // 单个描述子 -> word

// ---- 相似度 ----
double score(const BowVector &a, const BowVector &b) const;  // 成对评分；ORB-SLAM 用它

// ---- 树结构导航（直接索引用）----
NodeId getParentNode(WordId wid, int levelsup) const;
void getWordsFromNode(NodeId nid, std::vector<WordId> &words) const;

// ---- 停用低权重词（不可逆）----
int stopWords(double minWeight);   // weight 置 0；再用更小阈值调用也恢复不了

// ---- 持久化 ----
// 原版 DBoW2
void save(const std::string &filename) const;   // cv::FileStorage（.yml.gz）
void load(const std::string &filename);
void save(cv::FileStorage &fs, const std::string &name = "vocabulary") const;
void load(const cv::FileStorage &fs, const std::string &name = "vocabulary");
// ORB-SLAM2/3 的魔改版额外提供（原版没有！）
bool loadFromTextFile(const std::string &filename);      // 读 ORBvoc.txt；失败返回 false
void saveToTextFile(const std::string &filename) const;
// DBoW3 额外提供
void save(const std::string &filename, bool binary_compressed = true) const;  // .yml→YAML，其他→二进制
void load(const std::string &filename);                 // 自动识别 二进制/文本/YAML

// ---- 调试 ----
friend std::ostream& operator<<(std::ostream&, const TemplatedVocabulary&);
// 打印: k, L, Weighting(tf-idf/tf/idf/binary), Scoring(L1-norm/...), Number of words
```

---

## 2. DBoW2 `TemplatedDatabase<TDescriptor, F>`

```cpp
explicit TemplatedDatabase(bool use_di = true, int di_levels = 0);
template<class T> explicit TemplatedDatabase(const T &voc, bool use_di = true, int di_levels = 0);
// ⚠️ 构造时会【拷贝一份词典】(setVocabulary -> new T(voc))，内存翻倍

template<class T> void setVocabulary(const T &voc);                       // 会清空库
template<class T> void setVocabulary(const T& voc, bool use_di, int di_levels = 0);
const TemplatedVocabulary<TDescriptor,F>* getVocabulary() const;
void allocate(int nd = 0, int ni = 0);   // 预分配：nd 期望条目数, ni 每图词数

// ---- 入库 ----
EntryId add(const std::vector<TDescriptor> &features,
            BowVector *bowvec = NULL, FeatureVector *fvec = NULL);
EntryId add(const BowVector &vec, const FeatureVector &fec = FeatureVector());

// ---- 查询 ----
void query(const std::vector<TDescriptor> &features, QueryResults &ret,
           int max_results = 1, int max_id = -1) const;
void query(const BowVector &vec, QueryResults &ret,
           int max_results = 1, int max_id = -1) const;
//   max_results <= 0 表示全返回
//   max_id: 只接受 entry_id < max_id（⚠️ 源码严格小于，注释却写 <=）；-1 = 不限制

const FeatureVector& retrieveFeatures(EntryId id) const;   // 直接索引，需 use_di=true

// ---- 其他 ----
void clear();  unsigned int size() const;
bool usingDirectIndex() const;  int getDirectIndexLevels() const;
void save(const std::string &filename) const;   void load(const std::string &filename);
// 源码常量
static int MIN_COMMON_WORDS = 5;   // 仅作用于 CHI_SQUARE / BHATTACHARYYA 两条查询路径
```

---

## 3. `Result` / `QueryResults`（DBoW2 与 DBoW3 相同）

```cpp
typedef unsigned int EntryId;
class Result {
public:
  EntryId Id;          // 数据库条目 id
  double  Score;       // 相似度（方向见下表）
  int     nWords;      // 公共词数 ⚠️ 只在 CHI_SQUARE / BHATTACHARYYA 下填充
  double  bhatScore, chiScore;                  // 调试字段
  double  sumCommonVi, sumCommonWi, expectedChiScore;  // 调试字段
  bool operator<(const Result&) const;   // 比 Score
  static bool gt(const Result&, const Result&);   // a.Score > b.Score
  static bool geq(const Result&, const Result&);  // a.Score >= b.Score
  static bool ltId(const Result&, const Result&); // a.Id < b.Id
};
class QueryResults : public std::vector<Result> {
  void scaleScores(double factor);
  void saveM(const std::string &filename) const;   // 存 matlab
};
// query() 返回的结果已排好序：ret[0] 永远是最好的候选
```

---

## 4. 枚举与取值方向

```cpp
enum WeightingType { TF_IDF = 0, TF = 1, IDF = 2, BINARY = 3 };
enum ScoringType {
  L1_NORM = 0, L2_NORM = 1, CHI_SQUARE = 2, KL = 3, BHATTACHARYYA = 4, DOT_PRODUCT = 5
};
```

| ScoringType | Score 范围 | 方向 | 备注 |
|---|---|---|---|
| `L1_NORM` | [0,1] | 越大越好 | `1 - 0.5·‖v-w‖₁`；**ORBvoc.txt 用的就是它** |
| `L2_NORM` | [0,1] | 越大越好 | `1 - sqrt(1 - Σvᵢwᵢ)` |
| `CHI_SQUARE` | [0,1] | 越大越好 | 名字叫 distance，实为相似度；受 `MIN_COMMON_WORDS` 影响 |
| `KL` | 不可缩放 | **越小越好** | 唯一的反例；`Σvᵢln(vᵢ/wᵢ)` |
| `BHATTACHARYYA` | [0,1] | 越大越好 | `Σsqrt(vᵢwᵢ)`；受 `MIN_COMMON_WORDS` 影响 |
| `DOT_PRODUCT` | 不可缩放 | 越大越好 | `Σvᵢwᵢ` |

归一化行为：`L1_NORM` / `L2_NORM` 会把 BowVector 归一化；其余不归一化。
`transform()` 里 `TF_IDF` 每命中一次累加一次 idf（命中次数 = tf），最后按评分方式决定是否归一化。

`ORBvoc.txt` 首行 = `k L scoringType weightingType` = `10 6 0 0`
（⚠️ 与构造函数 `(k, L, weighting, scoring)` 的参数顺序**相反**）。
魔改版 `loadFromTextFile` 的合法范围：`k∈[0,20], L∈[1,10], scoring∈[0,5], weighting∈[0,3]`。

---

## 5. DBoW3

```cpp
#include "DBoW3/DBoW3.h"     // 非模板：没有 <TDescriptor, F>

// ---- Vocabulary ----
Vocabulary(int k = 10, int L = 5, WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);
Vocabulary(const std::string &filename);   Vocabulary(const char *filename);   Vocabulary(std::istream&);
void create(const std::vector<std::vector<cv::Mat> > &training_features);  // 外层=图像，内层=该图的描述子
void create(const std::vector<cv::Mat> &training_features);                // 每个元素是一个描述子
unsigned int size() const;   bool empty() const;   void clear();
int getBranchingFactor() const;  int getDepthLevels() const;  float getEffectiveLevels() const;
cv::Mat getWord(WordId wid) const;   WordValue getWordWeight(WordId wid) const;
WordId transform(const cv::Mat& feature) const;
void transform(const std::vector<cv::Mat>& features, BowVector &v) const;
void transform(const cv::Mat &features, BowVector &v) const;                     // ✨ 直接吃 Nx32 矩阵
void transform(const std::vector<cv::Mat>& features,
               BowVector &v, FeatureVector &fv, int levelsup) const;             // 回环检测用这个
double score(const BowVector &a, const BowVector &b) const;
NodeId getParentNode(WordId wid, int levelsup) const;
void getWordsFromNode(NodeId nid, std::vector<WordId> &words) const;
int stopWords(double minWeight);
void save(const std::string &filename, bool binary_compressed = true) const;
void load(const std::string &filename);       // 自动识别 二进制/文本/YAML；失败时 throw
int getDescritorSize() const;                 // ⚠️ 库里的拼写就是 Descritor
int getDescritorType() const;

// ---- Database ----
explicit Database(bool use_di = true, int di_levels = 0);
explicit Database(const Vocabulary &voc, bool use_di = true, int di_levels = 0);
void setVocabulary(const Vocabulary &voc);                                  // 会清空库
void setVocabulary(const Vocabulary& voc, bool use_di, int di_levels = 0);
void allocate(int nd = 0, int ni = 0);
EntryId add(const std::vector<cv::Mat> &features, BowVector *bowvec = NULL, FeatureVector *fvec = NULL);
EntryId add(const cv::Mat &features, BowVector *bowvec = NULL, FeatureVector *fvec = NULL);
EntryId add(const BowVector &vec, const FeatureVector &fec = FeatureVector());
void query(const std::vector<cv::Mat> &features, QueryResults &ret, int max_results = 1, int max_id = -1) const;
void query(const cv::Mat &features,              QueryResults &ret, int max_results = 1, int max_id = -1) const;
void query(const BowVector &vec,                 QueryResults &ret, int max_results = 1, int max_id = -1) const;
const FeatureVector& retrieveFeatures(EntryId id) const;
unsigned int size() const;   void clear();
bool usingDirectIndex() const;   int getDirectIndexLevels() const;
void save(const std::string &filename) const;   void load(const std::string &filename);
```

---

## 6. DBoW2 描述子策略：`FORB`

```cpp
class FORB : protected FClass {
public:
  typedef cv::Mat TDescriptor;        // 必须是 CV_8U
  typedef const TDescriptor *pDescriptor;
  static const int L = 32;            // 字节数 / 描述子长度
  static void meanValue(const std::vector<pDescriptor>&, TDescriptor &mean);  // 逐 bit 多数投票
  static double distance(const TDescriptor &a, const TDescriptor &b);         // 汉明距离
  static std::string toString(const TDescriptor &a);          // 文本词典用
  static void fromString(TDescriptor &a, const std::string &s);
  static void toMat32F(const std::vector<TDescriptor>&, cv::Mat &mat);   // Nx32 CV_32F
  static void toMat8U(const std::vector<TDescriptor>&, cv::Mat &mat);    // Nx32 CV_8U
};
// 同类兄弟: FBrief（BRIEF 描述子）、FSurf64（SURF-64，浮点）
```

---

## 7. ORB-SLAM2 里与之对接的关键点

```cpp
// include/ORBVocabulary.h
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

// Frame::ComputeBoW()
vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);  // Nx32 -> vector<cv::Mat>
mpORBVocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);              // levelsup = 4

// Converter::toDescriptorVector —— 等价实现
for (int j = 0; j < Descriptors.rows; j++) vDesc.push_back(Descriptors.row(j));

// KeyFrameDatabase —— 不用 DBoW2 的 TemplatedDatabase，自己维护 vector<list<KeyFrame*>> mvInvertedFile
mvInvertedFile.resize(voc.size());
for (auto vit = pKF->mBowVec.begin(); vit != pKF->mBowVec.end(); vit++)
    mvInvertedFile[vit->first].push_back(pKF);      // 存 KeyFrame*，不是 EntryId

// DetectLoopCandidates(pKF, minScore) 的关键常量（已核对源码）
//   spConnectedKeyFrames = pKF->GetConnectedKeyFrames();          // 排除共视帧
//   minCommonWords = maxCommonWords * 0.8f;                       // 自适应公共词门限
//   float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);         // 成对评分
//   if (si >= minScore) ...
//   vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);            // 共视簇累加分数
//   minScoreToRetain = 0.75f * bestAccScore;                      // 相对门限
// DetectRelocalizationCandidates(F): 不排除共视、无 minScore，门限同样 0.75f * bestAccScore
```

---

## 8. `BowVector` / `FeatureVector`（都是 `std::map` 的子类）

```cpp
// 注意：是 public 继承 std::map，所以 std::map 的迭代器/typedef 都能直接用
// （例如 DBoW2::FeatureVector::const_iterator），下面这些是额外加的辅助方法。
class BowVector : public std::map<WordId /*unsigned int*/, WordValue /*double*/> {
 public:
  void addWeight(WordId id, WordValue v);        // 存在则累加，不存在则插入（tf 就是这么累出来的）
  void addIfNotExist(WordId id, WordValue v);    // 只在不存在时插入（IDF / BINARY 用这个）
  void normalize(LNorm norm_type);               // LNorm 枚举: L1, L2
  void saveM(const std::string &filename, size_t W) const;   // 存 matlab
  friend std::ostream& operator<<(std::ostream &out, const BowVector &v);
};

class FeatureVector : public std::map<NodeId, std::vector<unsigned int> > {
 public:
  void addFeature(NodeId id, unsigned int i_feature);   // 直接索引就是靠它填出来的
  friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);
};

// 相关 typedef / 枚举（都在 DBoW2 命名空间，DBoW3 同名）
typedef unsigned int WordId;   typedef double WordValue;   typedef unsigned int NodeId;
enum LNorm { L1, L2 };
enum WeightingType { TF_IDF, TF, IDF, BINARY };            // 0,1,2,3
enum ScoringType { L1_NORM, L2_NORM, CHI_SQUARE, KL, BHATTACHARYYA, DOT_PRODUCT };  // 0..5
```

> 含义：一帧的 BowVector 之所以"稀疏"，是因为 `transform()` 只对**实际命中过的词**调用
> `addWeight`，没命中的词根本不在 map 里。这也是它比稠密直方图省内存、检索快的原因。
