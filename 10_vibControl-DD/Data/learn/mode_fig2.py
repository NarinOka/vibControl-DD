# result.csvを読み込んで，学習したモードおよび実際のモードを図で出力したい

from matplotlib import pyplot as plt
import pandas as pd
import os
import numpy as np

print(plt.style.available)
plt.style.use("classic")
plt.rcParams["font.size"] = 14
plt.rcParams["font.family"] = "Times New Roman"

unit_num: int = 6


def main():
    # filelist = os.listdir()
    # for file in filelist:
    #    if not os.path.isdir(file) and file[-4:]==".csv":
    #        csv2png(file)
    file = "00-03-46_result.csv"
    conditions = "00-03-46_lc.csv"
    csv2png(file, conditions)


def csv2png(file, conditions):
    result = pd.read_csv(file, index_col=0)
    condi = pd.read_csv(conditions, index_col=0, header=None)
    wave_num: int = int(condi.at["waveNum", 1])
    separators = np.arange(unit_num, unit_num * wave_num, unit_num)
    results = np.split(result, separators, axis=0)
    # waveNum = condi.at['waveNum', 1]
    # print(condi.at['frequency [Hz]', 1])    # 'frequency [Hz]'の1列目の値を取得
    # print(condi.at['waveNum', 1])    # 'waveNum'の1列目の値を取得
    # print(len(condi.columns))    # 列数を取得
    # print(condi.count(axis = 1))    # 各行(axis = 1)の要素数を取得
    # fig = []
    # wv = 0
    # while wv < waveNum:
    for wv_i in range(wave_num):
        fig = plt.figure(figsize=(4, 8))
        amp1 = fig.add_subplot(111)
        amp1.grid()

        amp1.set_xticks(np.arange(0, 2, 0.2))
        amp1.set_yticks(np.arange(0, unit_num, 1))
        amp1.set_xlim(0, 1)
        amp1.set_ylim(0, unit_num - 1)  # DOFをcsvに出力し，それを読み込むようにしたい

        amp1.set_xlabel("Vibration mode")
        amp1.set_ylabel("Degree of freedom")

        for i, data in results[wv_i].iteritems():
            # print(i,data)
            # plt.plot(result.index, data, label = i)
            amp1.plot(data, results[wv_i].index, label=i, marker="o")

            # if pd.isnull(result[]):
            #    wv += 1

        # plt.title(file)

        plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left", borderaxespad=0)
        fig.savefig("%s_wave%d.png" % (file, wv_i), bbox_inches="tight")  # グラフが見切れないように
        plt.close()


if __name__ == "__main__":
    main()
