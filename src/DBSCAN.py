# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import sklearn.datasets as ds
import matplotlib.colors
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler


def expand(a, b):
    d = (b - a) * 0.1
    return a-d, b+d


if __name__ == "__main__":
    f = open("../../semantic-SLAM/src/semantic_map.txt", "r")
    data = []
    for line in f.readlines():
    	line = line.strip('\n')
    	line = line.split(' ')
    	line.remove('')
    	if " " in line:
            line.remove(' ')
    	data.append(line)
    f.close()
    print(np.shape(data))
    data = np.array(data)  
    data, c = data[:,0::2], data[:,3]
    Data = []
    print(np.shape(data))
    for i in range(np.shape(data)[0]):
        for j in range(np.shape(data)[1]):
            data[i,j] = float(data[i,j])
    print(np.shape(data))
    for i in range(np.size(c)):
        if c[i] == '200':
            Data.append(data[i])
    Data = np.array(Data)
    print(np.shape(Data))
    Data = StandardScaler().fit_transform(Data)
    data = Data *1000
    print(np.shape(Data))
    print('Done.')
    # 数据1的参数：(epsilon, min_sample)
    params = ((200, 50), (200, 100), (225, 150), (130, 70), (100, 100), (100, 150))

    # 数据2
    # t = np.arange(0, 2*np.pi, 0.1)
    # data1 = np.vstack((np.cos(t), np.sin(t))).T
    # data2 = np.vstack((2*np.cos(t), 2*np.sin(t))).T
    # data3 = np.vstack((3*np.cos(t), 3*np.sin(t))).T
    # data = np.vstack((data1, data2, data3))
    # # # 数据2的参数：(epsilon, min_sample)
    # params = ((0.5, 3), (0.5, 5), (0.5, 10), (1., 3), (1., 10), (1., 20))

    matplotlib.rcParams['font.sans-serif'] = [u'Droid Sans Fallback']
    matplotlib.rcParams['axes.unicode_minus'] = False

    plt.figure(figsize=(24, 16), facecolor='w')
    plt.suptitle(u'DBSCAN clustering', fontsize=20)
    i = 2
    eps, min_samples = params[i]
    model = DBSCAN(eps=eps, min_samples=min_samples)
    model.fit(data)
    y_hat = model.labels_

    core_indices = np.zeros_like(y_hat, dtype=bool)
    core_indices[model.core_sample_indices_] = True

    y_unique = np.unique(y_hat)
    n_clusters = y_unique.size - (1 if -1 in y_hat else 0)## y_hat=-1为聚类后的噪声类
    print(y_unique, 'cluster number：', n_clusters)

    #plt.subplot(2, 3, i+1)
    clrs = plt.cm.Spectral(np.linspace(0, 0.8, y_unique.size))##指定聚类后每类的颜色
    print clrs
    for k, clr in zip(y_unique, clrs):
        cur = (y_hat == k)
        if k == -1:
            plt.scatter(data[cur, 0], data[cur, 1], s=20, c='k')
            continue
        plt.scatter(data[cur, 0], data[cur, 1], s=30, c=clr, edgecolors='k')
        #plt.scatter(data[cur & core_indices][:, 0], data[cur & core_indices][:, 1], s=60, c=clr, marker='o', edgecolors='k')
    x1_min, x2_min = np.min(data, axis=0) ## 两列的最小值
    x1_max, x2_max = np.max(data, axis=0)## 两列的最大值
    x1_min, x1_max = expand(x1_min, x1_max)
    x2_min, x2_max = expand(x2_min, x2_max)
    plt.xlim((x1_min, x1_max))
    plt.ylim((x2_min, x2_max))
    plt.grid(True)
    plt.title(ur'$\epsilon$ = %.1f  m = %d, cluster number:%d' % (eps, min_samples, n_clusters), fontsize=16)
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    plt.show()
