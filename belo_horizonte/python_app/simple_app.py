import numpy as np
import scipy as sp
import pandas as pd
import matplotlib.pyplot as plt

def main(input_file):
    # Importing the dataset
    dataset = pd.read_csv('Data.csv')
    X = dataset.iloc[:, :-1].values
    y = dataset.iloc[:, -1].values
    return dataset


if __name__ == "__main__":
    main(
        input_file="../example_data/example1.tsv",
    )
