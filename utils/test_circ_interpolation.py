import numpy as np
import matplotlib.pyplot as plt

def circular_interpolation(start, end, center, radius, direction='clockwise'):
    resolution = 0.1
    """
    Generates points along a circular arc with a specified resolution.

    Parameters:
    - start: tuple of (x, y) for the start point of the arc.
    - end: tuple of (x, y) for the end point of the arc.
    - center: tuple of (cx, cy) for the center of the circle.
    - radius: radius of the circle.
    - direction: 'clockwise' or 'counterclockwise' for the direction of the arc.
    - resolution: distance between consecutive points along the arc.

    Returns:
    - List of (x, y) tuples representing points along the arc.
    """
    # Convert inputs to numpy arrays for ease of calculations
    start = np.array(start)
    end = np.array(end)
    center = np.array(center)

    # Compute angles
    def angle_from_center(point):
        return np.arctan2(point[1] - center[1], point[0] - center[0])

    start_angle = angle_from_center(start)
    end_angle = angle_from_center(end)

    # Ensure angles are within the range [0, 2*pi)
    start_angle = start_angle % (2 * np.pi)
    end_angle = end_angle % (2 * np.pi)

    # Compute angular difference
    angular_range = end_angle - start_angle

    # Adjust angular_range for correct direction
    if direction == 'clockwise':
        if angular_range < 0:
            angular_range += 2 * np.pi
    else:
        if angular_range > 0:
            angular_range -= 2 * np.pi

    # Calculate the number of points required
    arc_length = abs(angular_range) * radius
    num_points = int(np.ceil(arc_length / resolution))

    # Generate points
    angles = np.linspace(start_angle, start_angle + angular_range, num_points)
    points = [(center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)) for angle in angles]

    return points

# Example usage
start_point = [1, 0]
end_point = [0, 1]
center_point = [0, 0]
radius = 1
direction = 'counterclockwise'
resolution = 0.1

points = circular_interpolation(start_point, end_point, center_point, radius, direction)

# Plotting the results
points = np.array(points)
fig, ax = plt.subplots()
circle = plt.Circle(center_point, radius, color='grey', fill=False, linestyle='--')
ax.add_artist(circle)
ax.plot(points[:, 0], points[:, 1], 'b-o', label='Arc Points')
ax.plot(*start_point, 'ro', label='Start Point')
ax.plot(*end_point, 'go', label='End Point')
ax.plot(*center_point, 'ko', label='Center Point')
ax.set_aspect('equal', 'box')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Circular Interpolation with Resolution')
ax.legend()
plt.grid(True)
plt.show()
