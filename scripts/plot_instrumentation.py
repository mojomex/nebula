#!/usr/bin/python3

import argparse
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import re
import json
import os

def condition_data(log_file: str):
  with open(log_file, 'r') as f:
    lines = f.readlines()
    lines = [re.search(r'(\{"tag":.*?\})', l) for l in lines]
    lines = [re.sub(r'([0-9])"', r'\1', l[1]) for l in lines if l]
    lines = [json.loads(l) for l in lines if l]
    df = pd.DataFrame(lines)

  df = df.groupby(['tag']).agg(del_min_min=('del_min', 'min'), del_avg_mean=('del_avg', 'mean'), del_max_max=('del_max', 'max'), rtt_min_min=('rtt_min', 'min'), rtt_avg_mean=('rtt_avg', 'mean'), rtt_max_max=('rtt_max', 'max'), frq_min_min=('frq_min', 'min'), frq_avg_mean=('frq_avg', 'mean'), frq_max_max=('frq_max', 'max'), window_mean=('window', 'mean')).reset_index()

  for col in df.columns:
      if col.startswith("del") or col.startswith("rtt"):
          df[col] *=1_000_000

  df = df.drop(columns=['frq_min_min'])
  df = df.drop(columns=['frq_avg_mean'])
  df = df.drop(columns=['frq_max_max'])

  df = df.rename(columns={'window_mean': 'frequency (avg) [Hz]'})
  df = df.rename(columns={'del_min_min': 'duration (min) [µs]'})
  df = df.rename(columns={'del_avg_mean': 'duration (avg) [µs]'})
  df = df.rename(columns={'del_max_max': 'duration (max) [µs]'})
  df = df.rename(columns={'rtt_min_min': 'rtt (min) [µs]'})
  df = df.rename(columns={'rtt_avg_mean': 'rtt (avg) [µs]'})
  df = df.rename(columns={'rtt_max_max': 'rtt (max) [µs]'})

  df = df.set_index('tag')

  dfT = df.T

  dfT_dur = dfT[[label.startswith('duration') for label in dfT.index]]
  dfT_rtt = dfT[[label.startswith('rtt') for label in dfT.index]]
  dfT_frq = dfT[[label.startswith('frequency') for label in dfT.index]]


  dfT_dur.attrs["title"] = "Duration"
  dfT_rtt.attrs["title"] = "Round-Trip Time"
  dfT_frq.attrs["title"] = "Frequency"

  dfT_frq.attrs["file"] = os.path.basename(os.path.splitext(log_file)[0])

  return (dfT_dur, dfT_rtt, dfT_frq)

def plot(conditioned_logs):
  fig, (ax0, ax1, ax2) = plt.subplots(1, 3, figsize=(15, 8), dpi=200)

  dfs0, dfs1, dfs2 = tuple(zip(*conditioned_logs))
  print(f"Got conditioned data from {len(dfs0)} logs")

  legend_handles = {}

  for (dfs, ax) in zip((dfs0, dfs1, dfs2), (ax0, ax1, ax2)):
    ax: plt.Axes
    ax.semilogy()
    ax.set_xticks(list(range(len(dfs[0].columns))), dfs[0].columns)
    ax.tick_params(axis='x', labelrotation=90)
    ax.set_title(dfs[0].attrs["title"])

    cycle = plt.get_cmap("tab10")
    for i, df in enumerate(dfs):
      clr = cycle(i)
      xs = np.array(range(len(df.columns))) - .4 + i * .1

      if (ax != ax2):
        min_col = [c for c in df.index if "(min)" in c][0]
        max_col = [c for c in df.index if "(max)" in c][0]
        ax.vlines(xs, df.loc[min_col], df.loc[max_col], color=clr)
        ax.scatter(xs, df.loc[min_col], marker="_", color=clr)
        ax.scatter(xs, df.loc[max_col], marker="_", color=clr)

      avg_col = [c for c in df.index if "(avg)" in c][0]
      handle = ax.scatter(xs, df.loc[avg_col], marker="D", color=clr)

      if (ax == ax2):
        legend_handles[df.attrs["file"]] = handle

  ax0.set_ylabel("Duration [µs]")
  ax1.set_ylabel("Round-Trip Time [µs]")
  ax2.set_ylabel("Frequency [Hz]")
  ax2.legend(legend_handles.values(), legend_handles.keys())

  fig.tight_layout()
  plt.savefig("instrumentation.png")
  plt.show()

def main(args):
   conditioned_logs = [condition_data(f) for f in args.log_files]
   plot(conditioned_logs)
  
if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("log_files", nargs="+")
  args = parser.parse_args()

  main(args)