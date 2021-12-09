import pandas as pd
import numpy as np
import scipy.stats
import argparse

parser = argparse.ArgumentParser(description='Recalculation of the stats computed by lidar calibration test pipeline using scipy and numpy')
parser.add_argument('csv_dir', type=str, help='csv that contains the stats.')

args = parser.parse_args()

t_cols = ["tx_err", "ty_err", "tz_err"]
r_cols = ["R_err", "P_err", "Y_err"]

df = pd.read_csv(args.csv_dir)

agents = list(df["agent_id"].unique())
tf_types = [t_cols, r_cols]
data_t = dict.fromkeys(agents)

for agent in agents:
    agent_dict = dict.fromkeys(t_cols+r_cols, 1)
    for tf_type in tf_types:
        for t in tf_type:
            agent_dict[t] = np.array(df[df["agent_id"] == agent][t].to_list())
            data_t[agent] = agent_dict

for agent in agents:
    print(f"For: {agent}")
    print(f"XYZ error(mean): {np.mean(data_t[agent]['tx_err']):.6f} {np.mean(data_t[agent]['ty_err']):.6f} {np.mean(data_t[agent]['tz_err']):.6f}") 
    print(f"RPY error(mean): {scipy.stats.circmean(data_t[agent]['R_err']):.6f} {scipy.stats.circmean(data_t[agent]['P_err']):.6f} {scipy.stats.circmean(data_t[agent]['Y_err']):.6f}")
    print(f"XYZ error (stdev): {np.std(data_t[agent]['tx_err']):.6f} {np.std(data_t[agent]['ty_err']):.6f} {np.std(data_t[agent]['tz_err']):.6f}")
    print(f"RPY error (stdev): {scipy.stats.circstd(data_t[agent]['R_err']):.6f} {scipy.stats.circstd(data_t[agent]['P_err']):.6f} {scipy.stats.circstd(data_t[agent]['Y_err']):.6f}")

    print("-"*20)

