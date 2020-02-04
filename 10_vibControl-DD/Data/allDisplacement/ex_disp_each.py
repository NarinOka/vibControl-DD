# https://github.com/YNakamura0528/csv2png_python
# https://qiita.com/miler0528/items/5ce48a1ecf8a752ed5ef
## https://qiita.com/kakiuchis/items/798c00f54c9151ab2e8b
## https://qiita.com/matsui-k20xx/items/291400ed56a39ed63462
## https://qiita.com/nanan/items/0e0e792b13541b0613b2 見切れる凡例について

from matplotlib import pyplot as plt
import pandas as pd
import os
import numpy as np

### 変位の時刻歴を別々のグラフに．

print(plt.style.available)
plt.style.use('classic')
plt.rcParams["font.size"] = 20
plt.rcParams['font.family'] = 'Times New Roman'

plot_styles: dict = {
    1: dict(color="red"),
    2: dict(color="green"),
    3: dict(color="blue"),
    4: dict(color="brown"),
    5: dict(color="magenta"),
}

oneFig = True # 全ての自由度のグラフを1つのfigにsubplotでまとめるかどうか
                # これに関わる部分をもっときれいにしたい
                # Falseの時が(gridとか)うまくいかない

def main():
    file = "10-25-15_freq2.56257.csv"
    csv2png(file)


def csv2png(file):
    df = pd.read_csv(file, index_col=0)
    pos_num: int = 6
    xmin, xmax = 0.0, 20.0
    ymin, ymax = -0.85, 0.85

    if oneFig:
        fig = plt.figure(figsize=(6, 12))
    else:
        fig = plt.figure(figsize=(8, 6))

    fig.subplots_adjust(hspace=0.0) # subplot間の間隔を整える http://ailaby.com/subplots_adjust/

    for pos_i in reversed(range(pos_num)):

        if pos_i == 0:
            continue    # Basement のグラフは出力しない

        # amp1 = fig.add_subplot(111)
        if oneFig:
            amp1 = fig.add_subplot(pos_num - 1, 1, pos_num - pos_i) # add_subplot(行,列,場所) https://teratail.com/questions/72586
        else:
            amp1 = fig.add_subplot(111)

        # plt.title(file)
        amp1.set_xticks(np.arange(0.0, 100.0, 2.0))
        amp1.set_yticks(np.arange(-10, 10, 0.2))
        amp1.set_xlim(xmin, xmax)
        amp1.set_ylim(ymin, ymax)
        amp1.grid() 
        amp1.vlines(5.0, ymin, ymax, linewidth=2.0, color = "red")
        amp1.vlines(10.0, ymin, ymax, linewidth=2.0, color = "blue")
        amp1.vlines(15.0, ymin, ymax, linewidth=2.0, color = "green")

        if oneFig:
            if pos_i == 1:    # 最後(Mass 1)だけxlabelつける
                amp1.set_xlabel('Simulation time [s]', fontsize = 24)
            if pos_i == pos_num/2:    # 真ん中だけylabelつける
                amp1.set_ylabel('Displacement [m]', fontsize = 24)
            if pos_i != 1:    # 最後(Mass 1)以外は目盛りをつけない https://qiita.com/tsukada_cs/items/8d31a25cd7c860690270
                amp1.tick_params(labelbottom = False)
        else:
            amp1.set_xlabel('Simulation time [s]', fontsize = 24)
            amp1.set_ylabel('Displacement [m]', fontsize = 24)


        for i, dat in df.iloc[:,[pos_i]].iteritems():
            # print(i)
            amp1.plot(df.index, dat, label = i, **plot_styles[pos_i]) # iは0行目の列番号(header)

        amp1.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize = 12)
        
        if not oneFig:
            fig.savefig(file+"_Mass" +str(pos_i)+".png", bbox_inches='tight', transparent = False) # グラフが見切れないように

    #plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize = 17)
    
    if oneFig:
        fig.savefig(file+"_oneFig"+".pdf", bbox_inches='tight', transparent = False) # グラフが見切れないように
    plt.close()


if __name__ == '__main__':
    main()