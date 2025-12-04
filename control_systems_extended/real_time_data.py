import math
import time
import sys
import cv2
import os
import pandas
from PIL import Image
import numpy as np



def newlatlon(lat , lon , hdg ,dist, movementHead):
    lati=math.radians(lat)
    longi=math.radians(lon)
    rade = 6367489
    AD = dist/rade
    sumofangles = (hdg + movementHead + 360)%360
    newheading = math.radians(sumofangles)
    newlati = math.asin(math.sin(lati)*math.cos(AD) + math.cos(lati)*math.sin(AD)*math.cos(newheading))
    newlongi = longi + math.atan2(math.sin(newheading)*math.sin(AD)*math.cos(lati), math.cos(AD)-math.sin(lati)*math.sin(newlati))
    ret_lat = int(round(math.degrees(newlati*1e7)))
    ret_lon = int(round(math.degrees(newlongi*1e7)))
    print("Latitude and Longitude in newlatlon:",ret_lat,"and",ret_lon,file=sys.stderr)
    return ret_lat, ret_lon

def distance_lat_lon(lat1, lon1, lat2, lon2):
    '''distance between two points'''
    dLat = math.radians(lat2) - math.radians(lat1)
    dLon = math.radians(lon2) - math.radians(lon1)
    a = math.sin(0.5*dLat)**2 + math.sin(0.5*dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(abs(a)), math.sqrt(abs(1.0-a)))
    ground_dist = 6371 * 1000 * c
    return ground_dist

def get_midpoint(image_path):
    # Open the image
    img = Image.open(image_path)
    # Get the width and height of the image
    width, height = img.size
    # Calculate the mid_x and mid_y coordinates
    mid_x = width // 2
    #print("in midpoint fxn",file=sys.stderr)
    mid_y = height // 2
    return (mid_x, mid_y)

def get_midpoint2(image):
    height, width, _ = image.shape
    midpoint = (width // 2, height // 2)
    return midpoint

# Function to plot lines based on yaw
def plot_lines_based_on_yaw(image, yaw, lati, longi, angle, lat_to_use, long_to_use):
    # Get the midpoint of the image
    midpoint = get_midpoint2(image)

    # Convert yaw to radians
    yaw_rad = math.radians(yaw)

    # Calculate the line passing through the midpoint with the given yaw
    line_length = 1000
    line_dx = int(line_length * math.cos(yaw_rad))
    line_dy = int(line_length * math.sin(yaw_rad))
    line_start = (midpoint[0] - line_dx, midpoint[1] - line_dy)
    line_end = (midpoint[0] + line_dx, midpoint[1] + line_dy)

    # Calculate the perpendicular line
    perp_dx = int(line_length * math.cos(yaw_rad + math.pi / 2))
    perp_dy = int(line_length * math.sin(yaw_rad + math.pi / 2))
    perp_start = (midpoint[0] - perp_dx, midpoint[1] - perp_dy)
    perp_end = (midpoint[0] + perp_dx, midpoint[1] + perp_dy)

    # Convert the points to integers
    line_start = tuple(map(int, line_start))
    line_end = tuple(map(int, line_end))
    perp_start = tuple(map(int, perp_start))
    perp_end = tuple(map(int, perp_end))

    # Plot the lines on the image
    cv2.line(image, line_start, line_end, (0, 255, 0), 2)  # Green line for yaw
    cv2.line(image, perp_start, perp_end, (0, 0, 255), 2)  # Red line perpendicular to yaw

    cv2.putText(image, f"Lat:{lati},Lon:{longi},angle:{angle},Dist: {distance_lat_lon(lat_to_use, long_to_use, lati, longi)}m", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("Image with Real-Time Data", image)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()

# Function to update and display real-time data
def update_data(event, x, y, flags, param):
    time.sleep(0.2)
    if event == cv2.EVENT_MOUSEMOVE:
        # Do your real-time data calculation based on mouse position
        point2 = get_midpoint(image_path)
        point1 = (int(x/scale), int(y/scale))

        # calculate the distance between the two points in pixels
        pixel_distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        # calculate the angle between the two points in radians
        angle = (math.degrees((math.atan2(point2[1] - point1[1], point2[0] - point1[0]))) -90 + 360) % 360
        theta = math.degrees((math.atan2(point2[1] - point1[1], point2[0] - point1[0])))

        if theta < 0:
            theta = 360 + theta

        # convert the pixel distance to real-life distance using the GSD constant
        real_distance_gsd = (pixel_distance * GSD) / 100

        lat_to_use, long_to_use, alt_to_use, yaw, heading_to_use = row['lat'], row['lon'], row['alt'], row['yaw'], row['head']
        # Old method
        value = newlatlon(lat_to_use, long_to_use, yaw, real_distance_gsd, angle)
        lati = value[0] / 1e7
        longi = value[1] / 1e7

        # Display the updated image with real-time data
        updated_image = resized_image.copy()

        # Plot lines based on yaw
        plot_lines_based_on_yaw(updated_image, 30, lati, longi, angle, lat_to_use, long_to_use)


if __name__ == "__main__":
    # Load the image
    image_path = "/home/nerdnhk/Jet_download/test_cam_4/images/nvcamtest_8766_s00_00127.jpg"  # Replace with the actual path to your image
    csv_path = '/home/nerdnhk/Jet_download/test_cam_4/results.csv'
    image_name = os.path.basename(image_path)
    index = int(image_name[-9:-4])
    image = cv2.imread(image_path)

    # GSD 
    GSD = 0.36

    # Read CSV file
    df = pandas.read_csv(csv_path)
    row = df.loc[index]

    # Get the screen dimensions
    screen_width, screen_height = 1600, 926  # Replace with your actual screen resolution

    # Calculate the scaling factors
    scale_x = screen_width / image.shape[1]
    scale_y = screen_height / image.shape[0]

    # Choose the smaller scaling factor to maintain the original resolution
    scale = min(scale_x, scale_y)

    # Resize the image
    resized_image = cv2.resize(image, None, fx=scale, fy=scale)

    # Create a window and set the callback function for mouse events
    cv2.namedWindow("Image with Real-Time Data")
    cv2.setMouseCallback("Image with Real-Time Data", update_data)

    # Display the initial image
    cv2.imshow("Image with Real-Time Data", resized_image)

    # Wait for a key event to close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()
