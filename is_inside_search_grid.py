from shapely.geometry import Polygon, Point
import folium

def is_point_inside_polygon(point, polygon_points):
    poly = Polygon(polygon_points)
    point = Point(point)
    return poly.contains(point)

def visualize_polygon_and_point(polygon_points, point):
    # Create a folium map centered at the first point in the polygon
    map_center = (polygon_points[0][0], polygon_points[0][1])
    my_map = folium.Map(location=map_center, zoom_start=12)

    # Add the polygon to the map
    folium.Polygon(locations=polygon_points, color='blue').add_to(my_map)

    # Add the point to the map
    folium.Marker(location=point, popup='Point', icon=folium.Icon(color='red')).add_to(my_map)

    # Save the map to an HTML file (optional)
    my_map.save('polygon_map.html')

    # Display the map
    my_map

# Example usage:
polygon_points_complex = [
    (38.31442311312976, -76.54522971451763),
    (38.31421041772561, -76.54400246436776),
    (38.31440703962630, -76.54394394383165),
    (38.31461622313521, -76.54516993186949),
    (38.31442311312976, -76.54522971451763)
]

point_to_check_complex = (7.7799, -122.4094)

result_complex = is_point_inside_polygon(point_to_check_complex, polygon_points_complex)
print(f"Is the point inside the complex polygon? {result_complex}")

# Visualize the complex polygon and point on the map
visualize_polygon_and_point(polygon_points_complex, point_to_check_complex)