# https://github.com/YNakamura0528/csv2png_python
# https://qiita.com/miler0528/items/5ce48a1ecf8a752ed5ef
## https://qiita.com/kakiuchis/items/798c00f54c9151ab2e8b
## https://qiita.com/matsui-k20xx/items/291400ed56a39ed63462
## https://qiita.com/nanan/items/0e0e792b13541b0613b2 見切れる凡例について

from matplotlib import pyplot as plt
import pandas as pd
import os

### added by me ###
print(plt.style.available)
plt.style.use('classic')
plt.rcParams['font.family'] = 'Times New Roman'
### ###

def main():
    filelist = os.listdir()
    for file in filelist:
        if not os.path.isdir(file) and file[-4:]==".csv":
            csv2png(file)


def csv2png(file):
    df = pd.read_csv(file, index_col=0)
    for i, dat in df.iteritems():
        # print(i,dat)
        plt.plot(df.index, dat, label = i)
    # plt.title(file)
    ### added by me ###
    plt.grid() 
    plt.xlim(0.0,300.0)
    plt.ylim(-0.5,0.5)
    plt.xlabel('simulation time [s]')
    plt.ylabel('displacement [m]')
    ### ###

    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)
    plt.savefig(file+".png", bbox_inches='tight', transparent = True) # グラフが見切れないように
    plt.close()


if __name__ == '__main__':
    main()