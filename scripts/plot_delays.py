#!/usr/bin/python3

import argparse
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import re

parser = argparse.ArgumentParser()
parser.add_argument("log_file")
args = parser.parse_args()

with open(args.log_file, 'r') as f:
  lines = f.readlines()
  lines = [re.search("#([A-Za-z]{3}) ([0-9]+\.[0-9]+)ms", l) for l in lines]
  lines = [l for l in lines if l]
  lines = [(m[1], float(m[2])) for m in lines]
  df = pd.DataFrame(lines, columns=["tag", 'dt'])

print(df.groupby("tag").describe())
