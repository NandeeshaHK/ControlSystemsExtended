import math
import cv2
import pandas as pd
import os
from ..utils.math import calculate_distance_global, new_lat_lon

def process_image_data(image_path, csv_path, gsd=0.36):
    """
    Process image data to calculate real-world coordinates from pixel data.
    """
    if not os.path.exists(image_path) or not os.path.exists(csv_path):
        print("Image or CSV file not found.")
        return

    image_name = os.path.basename(image_path)
    try:
        index = int(image_name[-9:-4])
    except ValueError:
        print("Could not extract index from image name.")
        return

    image = cv2.imread(image_path)
    df = pd.read_csv(csv_path)
    
    if index >= len(df):
        print("Index out of bounds in CSV.")
        return

    row = df.loc[index]
    
    # ... (rest of the logic from real_time_data.py adapted for SDK usage)
    # For now, we'll expose the core calculation logic as a reusable function
    return row, image

def calculate_target_lat_lon(click_x, click_y, image_shape, screen_dims, drone_telemetry, gsd=0.36):
    """
    Calculate target lat/lon based on a click in the image.
    """
    screen_width, screen_height = screen_dims
    scale_x = screen_width / image_shape[1]
    scale_y = screen_height / image_shape[0]
    scale = min(scale_x, scale_y)

    mid_x = image_shape[1] // 2
    mid_y = image_shape[0] // 2
    
    # Adjust click coordinates to image coordinates
    img_click_x = int(click_x / scale)
    img_click_y = int(click_y / scale)

    pixel_distance = math.sqrt((mid_x - img_click_x)**2 + (mid_y - img_click_y)**2)
    angle = (math.degrees((math.atan2(mid_y - img_click_y, mid_x - img_click_x))) - 90 + 360) % 360
    
    real_distance = (pixel_distance * gsd) / 100
    
    lat, lon, _, yaw, heading = drone_telemetry
    
    # Use new_lat_lon from utils
    new_lat, new_lon = new_lat_lon(lat, lon, yaw, real_distance, angle)
    
    return new_lat / 1e7, new_lon / 1e7
