# SLAM 中的 BoW（视觉词袋）学习包

给"正在熟悉 SLAM 常用库和函数"准备的 BoW 学习与使用指南：原理 → 库谱系 → API → 工作流 → 源码导读 → 调参 → 踩坑 → 自测实验。

## 从哪开始

1. **[`BoW库学习与使用指南.md`](BoW库学习与使用指南.md)** —— 主文档，先读 §0 和 §2。
2. **[`examples/bow_from_scratch.cpp`](examples/bow_from_scratch.cpp)** —— 纯标准库的最小 BoW 实现，
   10 分钟跑通就能建立直觉（层次 k-means 词典 + TF-IDF + 倒排索引 + L1 评分，语义对齐 DBoW2）：
   ```bash
   cd examples
   g++ -std=c++11 -O2 -o bow_from_scratch bow_from_scratch.cpp
   ./bow_from_scratch
   ```
   预期输出见 `examples/bow_from_scratch_expected_output.txt`。
3. **[`BoW-API速查表.md`](BoW-API速查表.md)** —— 写代码时查签名用。
4. 有 OpenCV/DBoW 环境后，跑 `examples/dbow2_orb_loop_demo.cpp`（DBoW2 + ORBvoc.txt）或
   `examples/dbow3_train_and_query.cpp`（DBoW3 训练 + 检索）。

## 验证状态（重要）

| 文件 | 状态 |
|---|---|
| `examples/bow_from_scratch.cpp` | ✅ 已用 g++ 8.1.0 `-std=c++11 -O2` 编译并运行，输出已保存 |
| `examples/bow_from_scratch_expected_output.txt` | ✅ 真实运行输出 |
| `examples/dbow2_orb_loop_demo.cpp` | ⚠️ 无真实 OpenCV/DBoW2 环境，未链接运行；已用 `.syntaxcheck/` 的桩头文件通过 `-Wall -Wextra` 语法检查，API 签名/常量逐个对照过源码 |
| `examples/dbow3_train_and_query.cpp` | ⚠️ 同上 |
| `examples/dbow3_python_quickstart.py` | ⚠️ 本机无 Python，未运行 |
| `examples/CMakeLists.txt` | ⚠️ 本机无 cmake，未验证 |

本机环境：MinGW g++ 8.1.0；无 cmake / OpenCV / Python。

`.syntaxcheck/` 是**只用于语法检查的桩头文件**（不是真实 OpenCV/DBoW），这样在没有 OpenCV 的机器上也能
校验示例代码的语法与接口调用：

```bash
cd examples
g++ -std=c++11 -fsyntax-only -Wall -Wextra -I ../.syntaxcheck dbow2_orb_loop_demo.cpp
g++ -std=c++11 -fsyntax-only -Wall -Wextra -I ../.syntaxcheck dbow3_train_and_query.cpp
```

它们按已核实的真实头文件建模，**不能**用来链接运行；真机编译请按各 `.cpp` 头部的命令。

## 最短结论（如果只记一件事）

BoW 只负责**快速产出回环候选**；回环是否成立必须靠几何验证（PnP/Sim3 + 位姿图优化）。
读 ORB-SLAM 时：`KeyFrameDatabase::DetectLoopCandidates` 出候选 → `LoopClosing` 做匹配与 Sim3 → 优化器才是判定者。
