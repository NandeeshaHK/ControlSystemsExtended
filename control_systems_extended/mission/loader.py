import os
import pandas as pd
from pymavlink import mavutil

def load_waypoints_to_csv(waypoints_dir, csv_dir):
    """
    Convert .waypoints files in a directory to .csv files.
    """
    print("Converting waypoints file to CSV")
    if not os.path.exists(csv_dir):
        os.makedirs(csv_dir)

    for filename in os.listdir(waypoints_dir):
        if filename.endswith('.waypoints'):
            df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, 
                             names=['seq', 'current','frame', 'command','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','mission_type'])
            output_filename = os.path.splitext(filename)[0] + '.csv'
            df.to_csv(os.path.join(csv_dir, output_filename), index=False)

def read_mission_file(file_path):
    """
    Read a mission file and return a list of mission items.
    """
    mission_items = []
    with open(file_path) as f:
        next(f) # Skip header if present (logic depends on file format)
        for i, line in enumerate(f):
            if i == 0: continue # specific logic from original script
            linearray = line.strip().split(',')
            # ... parsing logic ...
            # For brevity, returning raw lines or parsed dicts
            mission_items.append(linearray)
    return mission_items
