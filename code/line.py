"""
Author: Xiang Lei
Created: 2023-09-27 21:09:30
Purpose: 
"""
import numpy as np
import matplotlib.pyplot as plt

# 线段：(x1, y1) -> (x2, y2)


def DDA(point1, point2):
    """数值微分法（DDA 算法）
    只能画 |k| <= 1，超过则会出现离散的点，失真
    优点：简单直观，迭代的算法
    缺点：浮点运算，每步都需四舍五入取整

    Args:
        point1 (_type_): _description_
        point2 (_type_): _description_

    Returns:
        _type_: _description_
    """
    x1, y1 = point1
    x2, y2 = point2
    k = (y2 - y1) / (x2 - x1)  # 求斜率

    points = []

    y = y1
    for x in range(x1, x2+1):
        points.append([x, int(y + 0.5)])  # 四舍五入，取整
        y += k

    return points


def midpoint_line(point1, point2):
    """中点画线法
    计算量太太大，不如DDA算法；只能绘制斜率在0到1之间的正斜率直线

    直线一般式：ax + by + c = 0
    ax + by + c > 0 --- 直线上方
    ax + by + c < 0 --- 直线下方

    构造判别式 d = F(xi+1,yi+0.5) = a(xi+1)+b(yi+0.5)+c
    当 b > 0 时，d = a(xi+1)+b(yi+0.5)+c > 0 时，选择下面的点
    当 b > 0 时，d = a(xi+1)+b(yi+0.5)+c < 0 时，选择上面的点

    Args:
        point1 (_type_): _description_
        point2 (_type_): _description_

    Returns:
        _type_: _description_
    """
    x1, y1 = point1
    x2, y2 = point2

    dy = y2 - y1
    dx = x2 - x1
    a = -dy
    b = dx  # 请保证一般式中 y 的系数大于 0
    # c = x2 * y1 - y2 * x1

    points = []
    points.append([x1, y1])

    x = x1
    y = y1
    while x < x2:
        x += 1

        d = a * (x+1) + b * (y+0.5)  # 计算第一个 d=a(x+1)+b(y+0.5)+c
        if d < 0:  # 选择上面
            y += 1
        points.append([x, y])

    return points


def midpoint_line_improve(point1, point2):
    """中点画线法的改进版

    根据上一次计算的 d 值判断接下来 d 值的变化

    Args:
        point1 (_type_): _description_
        point2 (_type_): _description_

    Returns:
        _type_: _description_
    """

    x1, y1 = point1
    x2, y2 = point2

    dy = y2 - y1
    dx = x2 - x1
    a = -dy
    b = dx  # 请保证一般式中 y 的系数大于 0
    # c = x2 * y1 - y2 * x1

    points = []
    points.append([x1, y1])

    x = x1
    y = y1

    d = a + b/2  # # 计算第一个 d=a(x+1)+b(y+0.5)+c
    while x < x2:
        x += 1
        # 根据 d 判断在直线的上方还是下方，跟 y 的系数正负有关！！！
        if d >= 0:  # 选择下面
            d += a  # 增量：a = d_new - d_old
        else:  # 选择上面
            d += (a+b)  # 增量：a+b = d_new - d_old
            y += 1
        points.append([x, y])

    return points


def bresenham_line(point1, point2):
    """_summary_
    由误差项 e 的正负，判断下一个点的位置

    Args:
        point1 (_type_): _description_
        point2 (_type_): _description_
    """
    x1, y1 = point1
    x2, y2 = point2

    dy = y2 - y1
    dx = x2 - x1
    k = dy / dx

    points = []
    points.append([x1, y1])

    x = x1
    y = y1

    e = -0.5  # 初值 e
    for _ in range(dx):
        x += 1
        e += k
        if e >= 0:
            y += 1
            e -= 1
        points.append((x, y))

    return points


def draw_line(points):
    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]

    plt.plot(x_values, y_values, marker='o', linestyle='-')
    plt.xlabel('X')
    plt.ylabel('Y')

    plt.xlim(0, 100)
    plt.ylim(0, 100)

    plt.gca().set_aspect('equal', adjustable='box')  # 等比例坐标轴

    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    point1 = [2, 2]
    point2 = [100, 40]

    # draw_line(DDA(point1, point2))
    # draw_line(midpoint_line(point1, point2))
    draw_line(bresenham_line(point1, point2))
