mport pandas as pd
import numpy as np
from visibility_planner import compute_global_path


df = pd.read_csv("CSV_Data/Simulation_Data_VG_V1.0_13.11.25.csv", sep=";")
map_array = df[["id", "type", "x", "y"]].to_numpy(dtype=object)

global_path = compute_global_path(map_array, epsilon_mm=20.0)
print(global_path)