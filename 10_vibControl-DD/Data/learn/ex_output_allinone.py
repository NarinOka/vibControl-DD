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
    "output": dict(marker="+", color="orangered"),
    "teach": dict(marker="o", color="blue"),
    1: dict(marker="o", color="seagreen"),
    2: dict(marker="o", color="darkblue"),
    3: dict(marker="o", color="firebrick"),
}

# freqs = [2.5, 7.5, 11.7]

def main():
    file = "16-12-25_result.csv"
    conditions = "16-12-25_lc.csv"
    csv2png(file, conditions)


def csv2png(file, conditions):
    result = pd.read_csv(file, index_col=0)
    condi = pd.read_csv(conditions, index_col=0, header=None)
    wave_num: int = int(condi.at["waveNum", 1])

    freqs = []
    for wave in range(wave_num):
        freqs.append(condi.at["frequency [Hz]", wave + 1])

    separators = np.arange(unit_num, unit_num * wave_num, unit_num)
    results = np.split(result, separators, axis=0)

    fig = plt.figure(figsize=(4, 8))
    amp1 = fig.add_subplot(111)
    amp1.grid()

    for wv_i in range(wave_num):

        amp1.set_xticks(np.arange(0, 2, 0.2))
        amp1.set_yticks(np.arange(1, unit_num, 1)) # 縦軸の0を表示しないため1から
        amp1.set_xlim(0, 1)
        amp1.set_ylim(0, unit_num - 1)  # DOFをcsvに出力し，それを読み込むようにしたい

        amp1.set_xlabel("Normalized amplitude \n as the output from sensor network")
        amp1.set_ylabel("Sensor unit ID")

        for i, data in results[wv_i].iteritems():
            if i == "output":
                amp1.plot(data, results[wv_i].index, label = r"$f$ = " +  str(freqs[wv_i]) + " Hz", **plot_styles[wv_i + 1])

        # plt.title(file)

        # plt.legend(bbox_to_anchor=(0.0, 1.0), loc="lower left", borderaxespad=0, fontsize = 20)
        fig.savefig("%s_wave%d_o.png" % (file, wv_i), bbox_inches="tight")  # グラフが見切れないように
        plt.close()
    
    fig.legend(bbox_to_anchor=(0.98, 0.98), loc="lower right", borderaxespad=0, fontsize = 20)
    fig.savefig("%s_%d_output.png" % (file, wv_i+1), bbox_inches="tight")  # グラフが見切れないように
    plt.close()


if __name__ == "__main__":
    main()
