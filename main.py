from vpython import *
import math

# ----- Simulation Setup -----
scene.width = 800
scene.height = 600
scene.background = color.black
scene.center = vector(0, 0, 0)

# In our units:
#   Distance: AU
#   Mass: Solar mass
#   Time: year
# With these units the gravitational constant becomes:
G = 4 * math.pi**2  # ≈39.4784
dt = 0.001  # time step in years


# ----- Define a Body Class -----
class Body:
    def __init__(self, pos, vel, mass, radius, col):
        self.sphere = sphere(pos=pos, radius=radius, color=col, make_trail=True)
        self.mass = mass
        self.vel = vel


# ----- Create Bodies (Realistic Solar System) -----
bodies = []

# Sun
sun = Body(
    pos=vector(0, 0, 0), vel=vector(0, 0, 0), mass=1.0, radius=0.2, col=color.yellow
)
bodies.append(sun)


# Helper function: circular orbital speed v = sqrt(G / a)
def circular_speed(a):
    return math.sqrt(G / a)


# Mercury
mercury = Body(
    pos=vector(0.387, 0, 0),
    vel=vector(0, circular_speed(0.387), 0),
    mass=1.66e-7,
    radius=0.02,
    col=color.gray(0.5),
)
bodies.append(mercury)

# Venus
venus = Body(
    pos=vector(0.723, 0, 0),
    vel=vector(0, circular_speed(0.723), 0),
    mass=2.45e-6,
    radius=0.03,
    col=color.orange,
)
bodies.append(venus)

# Earth
earth = Body(
    pos=vector(1.0, 0, 0),
    vel=vector(0, circular_speed(1.0), 0),
    mass=3e-6,
    radius=0.03,
    col=color.blue,
)
bodies.append(earth)

# Mars
mars = Body(
    pos=vector(1.524, 0, 0),
    vel=vector(0, circular_speed(1.524), 0),
    mass=3.23e-7,
    radius=0.02,
    col=color.red,
)
bodies.append(mars)

# Jupiter
jupiter = Body(
    pos=vector(5.203, 0, 0),
    vel=vector(0, circular_speed(5.203), 0),
    mass=0.0009543,
    radius=0.1,
    col=color.orange,
)
bodies.append(jupiter)

# Saturn
# (Using a slightly different yellow so it is visually distinct from the Sun)
saturn = Body(
    pos=vector(9.537, 0, 0),
    vel=vector(0, circular_speed(9.537), 0),
    mass=0.0002857,
    radius=0.09,
    col=vector(1, 1, 0.6),
)
bodies.append(saturn)

# Uranus
uranus = Body(
    pos=vector(19.191, 0, 0),
    vel=vector(0, circular_speed(19.191), 0),
    mass=4.366e-5,
    radius=0.07,
    col=color.cyan,
)
bodies.append(uranus)

# Neptune
neptune = Body(
    pos=vector(30.07, 0, 0),
    vel=vector(0, circular_speed(30.07), 0),
    mass=5.151e-5,
    radius=0.07,
    col=color.blue,
)
bodies.append(neptune)

# ----- Create the Grid Visualization -----
# Adjust grid range to cover the outer solar system (Neptune ~30 AU)
grid_range = 35  # extent of grid in x and y directions
grid_step = 1  # spacing for grid lines
grid_scale = 0.005  # scale factor for the potential visualization


def potential_at_point(x, y):
    pot = 0
    # Sum contributions from every body; add a small offset (0.1) to avoid singularities.
    for body in bodies:
        r = sqrt((x - body.sphere.pos.x) ** 2 + (y - body.sphere.pos.y) ** 2 + 0.1)
        pot += -G * body.mass / r
    return pot


grid_lines = []
grid_coords = []

# Horizontal lines (constant y, varying x)
for y in range(-grid_range, grid_range + 1, grid_step):
    pts = []
    coords = []
    for x in range(-grid_range, grid_range + 1):
        z = potential_at_point(x, y) * grid_scale
        pt = vector(x, y, z)
        pts.append(pt)
        coords.append((x, y))
    grid_lines.append(curve(*pts, color=color.green, opacity=0.5))
    grid_coords.append(coords)

# Vertical lines (constant x, varying y)
for x in range(-grid_range, grid_range + 1, grid_step):
    pts = []
    coords = []
    for y in range(-grid_range, grid_range + 1):
        z = potential_at_point(x, y) * grid_scale
        pt = vector(x, y, z)
        pts.append(pt)
        coords.append((x, y))
    grid_lines.append(curve(*pts, color=color.green, opacity=0.5))
    grid_coords.append(coords)

# ----- Simulation Loop -----
while True:
    rate(1000)  # Increase iterations per second for smoother orbits

    # Update positions of each body using the net gravitational force
    for body in bodies:
        net_force = vector(0, 0, 0)
        for other in bodies:
            if other is body:
                continue
            r_vec = other.sphere.pos - body.sphere.pos
            r_mag = mag(r_vec)
            force_mag = G * body.mass * other.mass / r_mag**2
            net_force += force_mag * norm(r_vec)
        body.vel += (net_force / body.mass) * dt
    for body in bodies:
        body.sphere.pos += body.vel * dt

    # Update grid lines for the current gravitational potential
    for line_index, line in enumerate(grid_lines):
        coords = grid_coords[line_index]
        for i in range(line.npoints):
            x, y = coords[i]
            new_z = potential_at_point(x, y) * grid_scale
            line.modify(i, vector(x, y, new_z))
