# 光栅化三角形的抗锯齿实现

姓名：雷翔
学号：2053932
日期：2023.12.15

### 文件结构

```bash
.
├── CMakeLists.txt
├── README.md
├── Triangle.cpp
├── Triangle.hpp
├── cmake-build-debug
├── global.hpp
├── main.cpp
├── rasterizer.cpp
├── rasterizer.hpp
└── result # 存放实验结果
```

本项目基于 [GAMES101](https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html) 课程的光栅化渲染框架，旨在通过实现多重采样抗锯齿（MSAA）和超级采样抗锯齿（SSAA）来探索和理解抗锯齿技术。

## 抗锯齿技术

抗锯齿技术是渲染过程中用于减少图像锯齿现象的一系列方法，主要目的是在不增加过多计算负担的前提下，提高图像边缘的平滑度。本项目实现了两种主要的抗锯齿技术：超级采样抗锯齿（SSAA）和多重采样抗锯齿（MSAA）。

获取矫正深度的函数的错误原因已在代码对应部分说明！

### SSAA (超级采样抗锯齿)

在 `rasterize_triangle_with_ssaa` 函数中实现，具体来说，对每个原始像素内部的多个子像素进行采样，然后计算这些子像素的颜色平均值作为原始像素的颜色。

### MSAA (多重采样抗锯齿)

在 `rasterize_triangle_with_msaa1`和 `rasterize_triangle_with_msaa2` 函数中实现。MSAA 减少了 SSAA 的计算复杂度，只计算有几个采样点会被三角形覆盖，利用像素中心坐标计算一次颜色。

- `rasterize_triangle_with_msaa1` 在实现过程中出现了黑边的问题。这是因为在光栅化三角形时，仅考虑了背景色和当前三角形的颜色，未能充分考虑与其他三角形颜色的相互影响。
- `rasterize_triangle_with_msaa2` 通过改进算法，成功消除了黑边现象，实现了更加精准和美观的渲染效果。

## 结果

原始效果（无抗锯齿）:

![output_None](https://lei-1306809548.cos.ap-shanghai.myqcloud.com/Obsidian/output_None.png)

SSAA效果:

![output_SSAA](https://lei-1306809548.cos.ap-shanghai.myqcloud.com/Obsidian/output_SSAA.png)

MSAA效果（有黑边）:

![output_MSAA 有黑边](https://lei-1306809548.cos.ap-shanghai.myqcloud.com/Obsidian/output_MSAA%20%E6%9C%89%E9%BB%91%E8%BE%B9.png)

MSAA效果（无黑边）:

![output_MSAA 无黑边](https://lei-1306809548.cos.ap-shanghai.myqcloud.com/Obsidian/output_MSAA%20%E6%97%A0%E9%BB%91%E8%BE%B9.png)

## 参考

https://www.zhihu.com/question/20236638

如有任何问题或建议，请通过邮箱[2053932@tongji.edu.cn](mailto:2053932@tongji.edu.cn)与我联系。