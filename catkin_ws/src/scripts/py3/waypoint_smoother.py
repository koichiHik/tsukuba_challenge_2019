#!/usr/bin/env python

import argparse

import pandas as pd
import numpy as np

  
if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('-raw_waypoint', type=str)
  parser.add_argument('-out_waypoint', type=str)
  parser.add_argument('-window_size', type=int, default=9)
  args = parser.parse_args()

  #column_list = ["x", "y", "z", "yaw", "velocity"]
  cols = {'x':np.float64,'y':np.float64,'z':np.float64,'yaw':np.float64,'change_flag':np.int32}
  df = pd.read_csv(args.raw_waypoint, dtype=cols)

  df["cos_yaw"] = np.cos(df["yaw"])
  df["sin_yaw"] = np.sin(df["yaw"])

  # X. Smooting.
  df["smooth_x"] = df["x"].rolling(window=args.window_size, center=True).mean()
  df['smooth_x'] = df['smooth_x'].where(~df['smooth_x'].isnull(), df['x'])

  df["smooth_y"] = df["y"].rolling(window=args.window_size, center=True).mean()
  df['smooth_y'] = df['smooth_y'].where(~df['smooth_y'].isnull(), df['y'])

  df["smooth_z"] = df["z"].rolling(window=args.window_size, center=True).mean()
  df['smooth_z'] = df['smooth_z'].where(~df['smooth_z'].isnull(), df['z'])

  df["smooth_cos_yaw"] = df["cos_yaw"].rolling(window=args.window_size, center=True).mean()
  df["smooth_cos_yaw"] = df["smooth_cos_yaw"].where(~df["smooth_cos_yaw"].isnull(), df["cos_yaw"])

  df["smooth_sin_yaw"] = df["sin_yaw"].rolling(window=args.window_size, center=True).mean()
  df["smooth_sin_yaw"] = df["smooth_sin_yaw"].where(~df["smooth_sin_yaw"].isnull(), df["sin_yaw"])

  df["smooth_yaw"] = np.arctan2(df["smooth_sin_yaw"], df["smooth_cos_yaw"])

  new_cols = {'x':np.float64,'y':np.float64,'z':np.float64,'yaw':np.float64,'change_flag':np.int32, 'stop_flag':np.int32}
  out_df = pd.DataFrame()
  out_df["x"] = df["smooth_x"]
  out_df["y"] = df["smooth_y"]
  out_df["z"] = df["smooth_z"]
  out_df["yaw"] = df["smooth_yaw"]
  out_df["velocity"] = df["velocity"]
  out_df["change_flag"] = df["change_flag"]
  out_df["stop_flag"] = 0
  out_df["event_flag"] = 0

  out_df.to_csv(args.out_waypoint, index=False)