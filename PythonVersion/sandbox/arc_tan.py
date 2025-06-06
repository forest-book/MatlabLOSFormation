import numpy as np
import matplotlib.pyplot as plt

# パラメータの定義
k_p = 1.0  # 適当な比例ゲイン
k_s = 2.0  # 適当なスケーリング定数
v_l = 3.0  # リーダの速度（定数と仮定）

# tの代わりにl(t)を横軸とする（-10から10まで）
ell = np.linspace(-10, 10, 400)

# hat_v_l(t) は単純に1と仮定
hat_v_l = 1.0

# 式の計算
nu = v_l * (1 + (2 * k_p / np.pi) * np.arctan((ell * hat_v_l) / k_s))

# グラフ描画
plt.figure(figsize=(8, 5))
plt.plot(ell, nu, label=r'$\nu(t)$')
plt.xlabel(r'$\ell(t)$')
plt.ylabel(r'$\nu(t)$')
plt.title(r'$\nu(t) = v_l(t) \left[1 + \frac{2k_p}{\pi} \tan^{-1}\left(\frac{\ell(t) \cdot \hat{v}_l(t)}{k_s}\right)\right]$')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

t = np.linspace(-10, 10, 400)
y = (2 / np.pi) * np.arctan(t)
plt.figure(figsize=(8, 5))
plt.plot(t, y, label="arc_tan")
plt.xlabel('t')
plt.ylabel('y')
plt.title('arc_tan shape')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

