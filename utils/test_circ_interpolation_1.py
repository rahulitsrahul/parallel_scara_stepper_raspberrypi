import numpy as np
import matplotlib.pyplot as plt

def parse_gcode(gcode, resolution=0.1):
    """
    Parses G-code for circular interpolation and returns the points along the arc.
    
    Parameters:
    - gcode: A string containing the G-code command with start, end, and either center offsets (I/J) or radius (R).
    - resolution: Distance between consecutive points along the arc.

    Returns:
    - List of (x, y) tuples representing points along the arc.
    """
    components = gcode.split()
    command = components[0]  # G2 or G3
    x_end = float(components[1][1:])
    y_end = float(components[2][1:])
    
    if 'I' in gcode and 'J' in gcode:
        # Handle I/J case
        i = float(components[3][1:])
        j = float(components[4][1:])
        direction = 'cw' if command == 'G2' else 'ccw'
        
        # Start point assumed to be (0, 0)
        start = np.array([0, 0])
        end = np.array([x_end, y_end])
        center = np.array([i, j])
        absolute_center = start + center
        radius = np.linalg.norm(center)
        
        def circular_interpolation(start, end, center, radius, direction, resolution):
            def angle_from_center(point, center):
                return np.arctan2(point[1] - center[1], point[0] - center[0])
            
            start_angle = angle_from_center(start, center)
            end_angle = angle_from_center(end, center)
            start_angle = start_angle % (2 * np.pi)
            end_angle = end_angle % (2 * np.pi)
            
            def angular_range(start_angle, end_angle, direction):
                range_angle = end_angle - start_angle
                if direction == 'cw':
                    if range_angle < 0:
                        range_angle += 2 * np.pi
                else:
                    if range_angle > 0:
                        range_angle -= 2 * np.pi
                return range_angle
            
            angular_range_value = angular_range(start_angle, end_angle, direction)
            arc_length = abs(angular_range_value) * radius
            num_points = int(np.ceil(arc_length / resolution))
            angles = np.linspace(start_angle, start_angle + angular_range_value, num_points)
            points = [(center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)) for angle in angles]
            return points
        
        return circular_interpolation(start, end, absolute_center, radius, direction, resolution)
    
    elif 'R' in gcode:
        # Handle R case
        radius = float(components[3][1:])
        direction = 'cw' if command == 'G2' else 'ccw'
        
        # Compute center from end point and radius
        start = np.array([0, 0])
        end = np.array([x_end, y_end])
        distance = np.linalg.norm(end - start)
        
        if radius < distance / 2:
            raise ValueError(f"Invalid radius: {radius}. It must be greater than half of the distance between start and end points ({distance / 2}).")
        
        # Vector from start to end
        vec = end - start
        perp_vec = np.array([-vec[1], vec[0]])
        perp_vec = perp_vec / np.linalg.norm(perp_vec)
        height = np.sqrt(radius**2 - (distance / 2)**2)
        
        center1 = (start + end) / 2 + height * perp_vec
        center2 = (start + end) / 2 - height * perp_vec
        
        # Compute points for both possible centers
        def calculate_points(center):
            def angle_from_center(point, center):
                return np.arctan2(point[1] - center[1], point[0] - center[0])
            
            def angular_range(start_angle, end_angle, direction):
                range_angle = end_angle - start_angle
                if direction == 'cw':
                    if range_angle < 0:
                        range_angle += 2 * np.pi
                else:
                    if range_angle > 0:
                        range_angle -= 2 * np.pi
                return range_angle
            
            start_angle = angle_from_center(start, center)
            end_angle = angle_from_center(end, center)
            start_angle = start_angle % (2 * np.pi)
            end_angle = end_angle % (2 * np.pi)
            angular_range_value = angular_range(start_angle, end_angle, direction)
            arc_length = abs(angular_range_value) * radius
            num_points = int(np.ceil(arc_length / resolution))
            angles = np.linspace(start_angle, start_angle + angular_range_value, num_points)
            points = [(center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)) for angle in angles]
            return points
        
        # Calculate points for both centers and choose based on specific criteria
        points1 = calculate_points(center1)
        points2 = calculate_points(center2)
        
        # For simplicity, return points1 or choose based on additional criteria
        return points1
    
    else:
        raise ValueError("Invalid G-code format. Must include either I/J or R for circular interpolation.")

def plot_points(points, title="Circular Interpolation"):
    """
    Plots the points along with the circle center and radius.
    
    Parameters:
    - points: List of (x, y) tuples representing points along the arc.
    - title: Title of the plot.
    """
    points = np.array(points)
    plt.figure(figsize=(8, 8))
    plt.plot(points[:, 0], points[:, 1], marker='o', linestyle='-', markersize=3)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(title)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()

# Example usage
gcode_i_j = "G2 X0 Y10 I0 J10"  # Should generate an arc
gcode_r_valid = "G2 X10 Y10 R10"  # Valid R example

print("Points from I/J G-code:")
points_i_j = parse_gcode(gcode_i_j)
plot_points(points_i_j, title="Arc from I/J G-code")

print("\nPoints from Valid R G-code:")
points_r_valid = parse_gcode(gcode_r_valid)
plot_points(points_r_valid, title="Arc from Valid R G-code")
