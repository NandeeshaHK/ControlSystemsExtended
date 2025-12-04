from shapely.geometry import Polygon, Point
try:
    import folium
except ImportError:
    folium = None

def is_point_inside_polygon(point, polygon_points):
    """
    Check if a point (lat, lon) is inside a polygon defined by a list of points.
    """
    poly = Polygon(polygon_points)
    point = Point(point)
    return poly.contains(point)

def visualize_polygon_and_point(polygon_points, point, output_file='polygon_map.html'):
    """
    Visualize a polygon and a point on a map using folium.
    """
    if folium is None:
        print("Folium is not installed. Visualization skipped.")
        return

    # Create a folium map centered at the first point in the polygon
    map_center = (polygon_points[0][0], polygon_points[0][1])
    my_map = folium.Map(location=map_center, zoom_start=12)

    # Add the polygon to the map
    folium.Polygon(locations=polygon_points, color='blue').add_to(my_map)

    # Add the point to the map
    folium.Marker(location=point, popup='Point', icon=folium.Icon(color='red')).add_to(my_map)

    # Save the map to an HTML file
    my_map.save(output_file)
    print(f"Map saved to {output_file}")
