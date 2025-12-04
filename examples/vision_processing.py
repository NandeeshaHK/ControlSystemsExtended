from control_systems_extended.telemetry import vision

def main():
    # Mock data for demonstration
    image_path = "path/to/image.jpg"
    csv_path = "path/to/data.csv"
    
    # This would normally load the image and data
    # row, image = vision.process_image_data(image_path, csv_path)
    
    print("Vision processing example")
    print("Use vision.calculate_target_lat_lon() to get real-world coordinates from image clicks.")

if __name__ == "__main__":
    main()
