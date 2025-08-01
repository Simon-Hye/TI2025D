import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

def fit_and_plot(x, y):
    """
    使用线性回归拟合数据并绘图，返回斜率和截距。
    参数:
        x (array-like): 自变量数组
        y (array-like): 因变量数组
    返回:
        k, b: 拟合公式中的斜率和截距
    """
    x = np.array(x)
    y = np.array(y)
    x_reshaped = x.reshape(-1, 1)

    # 拟合模型
    model = LinearRegression()
    model.fit(x_reshaped, y)
    k = model.coef_[0]
    b = model.intercept_
    print(f"拟合公式: y = {k:.3f} * x + {b:.3f}")

    # 绘制拟合图像
    x_fit = np.linspace(min(x), max(x), 100).reshape(-1, 1)
    y_fit = model.predict(x_fit)

    plt.scatter(x, y, color='blue', label='Raw Data')
    plt.plot(x_fit, y_fit, color='red', label='Fit Curve')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Linear Regression Fit')
    plt.legend()
    plt.grid(True)
    plt.show()

    return k, b
# C-value for one type
x = np.array([1, 1, 2, 2, 5, 5])
y = np.array([106.948508, 107.530478, 147.750204, 148.698063, 281.053706, 283.552206])
# R-value
x = np.array([30, 30, 30, 5.2, 5.2, 5.2,33,51])
y = np.array([29.889999,29.879999, 29.900000,5.14,5.14,5.14,33,])
fit_and_plot(x,y);
