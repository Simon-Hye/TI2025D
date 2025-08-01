import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

x = np.array([1, 1, 2, 2, 5, 5])
y = np.array([106.948508, 107.530478, 147.750204, 148.698063, 281.053706, 283.552206])

x_reshaped = x.reshape(-1, 1)

# 创建并拟合线性模型
model = LinearRegression()
model.fit(x_reshaped, y)

# 拟合参数
k = model.coef_[0]
b = model.intercept_
print(f"y = {k:.3f} * x + {b:.3f}")

# 生成拟合线
x_fit = np.linspace(min(x), max(x), 100).reshape(-1, 1)
y_fit = model.predict(x_fit)

# 绘图
plt.scatter(x, y, color='blue', label='Raw Data')
plt.plot(x_fit, y_fit, color='red', label='fit curve')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Linear Regression')
plt.legend()
plt.grid(True)
plt.show()
