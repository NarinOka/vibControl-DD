# result.csvを読み込んで，学習したモードおよび実際のモードを図で出力したい

from matplotlib import pyplot as plt
import pandas as pd
import os
import numpy as np

print(plt.style.available)
plt.style.use("classic")
plt.rcParams["font.size"] = 24
plt.rcParams["font.family"] = "Times New Roman"

unit_num: int = 6
plot_styles: dict = {
    1: dict(marker="+", color="red"),
    2: dict(marker="o", color="blue"),
    3: dict(marker="x", color="green"),
}


def main():
    file = "21-23-49_freq2.5_modeChange.csv"
    csv2png(file)


def csv2png(file):
    result = pd.read_csv(file, index_col=0)
    act_num: int = 3
    separators = np.arange(unit_num, unit_num * act_num, unit_num)
    results = np.split(result, separators, axis=0)
    # print(results[2].index[5])

    fig = plt.figure(figsize=(4, 8))
    amp1 = fig.add_subplot(111)
    amp1.grid()

    for wv_i in range(act_num):

        amp1.set_xticks(np.arange(0, 2, 0.1))
        amp1.set_yticks(np.arange(1, unit_num, 1)) # 縦軸の0を表示しないため1から
        amp1.set_xlim(0.0, 0.3)
        amp1.set_ylim(0.0, unit_num - 1)  # DOFをcsvに出力し，それを読み込むようにしたい

        amp1.set_xlabel("Maximum displacement [m]")
        amp1.set_ylabel("Degree of freedom [-]")

        for i, data in results[wv_i].iteritems():
            amp1.plot(data, results[wv_i].index, label= "Before actuator " + str(wv_i + 1) + " moved", **plot_styles[wv_i + 1])

        
    #fig.legend(bbox_to_anchor=(1.05, 0.94), loc="lower right", borderaxespad=0, fontsize = 20)

    fig.savefig("%s.png" % (file), bbox_inches="tight", pad_inches=0.2)  
    #  bbox_inches="tight" : グラフが見切れないように， pad_inches=0.2 : グラフ余白　https://qiita.com/MENDY/items/fe9b0c50383d8b2fd919
    plt.close()


if __name__ == "__main__":
    main()
