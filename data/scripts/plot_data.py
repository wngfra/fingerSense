import pandas as pd
import matplotlib.pyplot as plt

force = 5.0
speed = -0.01


def main():
    import sys

    try:
        filename = str(sys.argv[1])
        df = pd.read_csv(filename)
    except:
        print(f"Cannot find the data file {filename}!")
        return

    plt.plot(df.values)
    plt.title(filename)
    plt.show()


if __name__ == '__main__':
    main()
