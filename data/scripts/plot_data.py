import pandas as pd
import matplotlib.pyplot as plt

force = 5.0
speed = -0.01


def main():
    import sys

    try:
        force = abs(float(sys.argv[1]))
        speed = -abs(float(sys.argv[2]))
        filename = f'../BrownPolymer_{force}_{speed}.csv'
        df = pd.read_csv(filename)
    except:
        print(f"Cannot find the data file {filename}!")
        return

    plt.plot(df.values)
    plt.show()


if __name__ == '__main__':
    main()
