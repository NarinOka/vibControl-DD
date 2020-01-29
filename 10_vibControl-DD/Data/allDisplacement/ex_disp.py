# https://github.com/YNakamura0528/csv2png_python
# https://qiita.com/miler0528/items/5ce48a1ecf8a752ed5ef
## https://qiita.com/kakiuchis/items/798c00f54c9151ab2e8b
## https://qiita.com/matsui-k20xx/items/291400ed56a39ed63462
## https://qiita.com/nanan/items/0e0e792b13541b0613b2 見切れる凡例について

from matplotlib import pyplot as plt
import pandas as pd
import os
import numpy as np

### 全ての変位の時刻歴をひとつのグラフに．

### added by me ###
print(plt.style.available)
plt.style.use('classic')
plt.rcParams["font.size"] = 20
plt.rcParams['font.family'] = 'Times New Roman'
### ###

def main():
    file = "21-25-26_freq11.7.csv"
    csv2png(file)


def csv2png(file):
    df = pd.read_csv(file, index_col=0)
    for i, dat in df.iteritems():
        # print(i)
        # print(dat)
        # if(i=="Mass 1"):
        #     plt.plot(df.index, dat, label = i) # iは0行目の列番号(header)
        plt.plot(df.index, dat, label = i) # iは0行目の列番号(header)
    
    # plt.title(file)
    plt.grid() 
    plt.xticks(np.arange(0.0, 20.0, 1.0))
    plt.yticks(np.arange(-0.5, 0.5, 0.1))
    plt.xlim(0.0,8.0)
    plt.ylim(-0.35,0.35)
    plt.xlabel('Simulation time [s]', fontsize = 20)
    plt.ylabel('Displacement [m]', fontsize = 20)
    plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize = 17)
    plt.savefig(file+".png", bbox_inches='tight', transparent = False) # グラフが見切れないように

    #plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize = 17)
    #plt.savefig(file+".png", bbox_inches='tight', transparent = False) # グラフが見切れないように
    plt.close()


if __name__ == '__main__':
    main()