import numpy as np
import cv2

# --- Load & prep map ---
map_img = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)  
if map_img is None:
    raise FileNotFoundError("Could not read map.png")
HEIGHT, WIDTH = map_img.shape

map_blur = cv2.GaussianBlur(map_img, (7, 7), 0)

# Robot pose (floats)
rx, ry, rtheta = (WIDTH / 4.0, HEIGHT / 4.0, 0.0)

# Tunables
STEP = 5.0
TURN = np.radians(25.0)

SIGMA_STEP = 0.5
SIGMA_TURN = np.radians(5.0)

NUM_PARTICLES = 3000

SHOW_PARTICLES = False


# --- Input ---
def get_input():

    k = cv2.waitKeyEx(0)

    fwd = 0.0
    turn = 0.0
    halt = False
    toggle = False

    # Movement keys
    if k in (2490368, 0x260000):  # Up
        fwd = STEP
    elif k in (2555904, 0x270000):  # Right
        turn = TURN
    elif k in (2424832, 0x250000):  # Left
        turn = -TURN
    elif k in (ord('w'), ord('W')):
        fwd = STEP
    elif k in (ord('d'), ord('D')):
        turn = TURN
    elif k in (ord('a'), ord('A')):
        turn = -TURN
    elif k in (ord('p'), ord('P')):
        toggle = True
    elif k in (ord('q'), 27):  # q or Esc
        halt = True
    

    return fwd, turn, halt, toggle


# --- Motion models ---
def move_robot(rx, ry, rtheta, fwd, turn):
    
    # Add noise
    fwd_noisy = fwd + float(np.random.normal(0.0, SIGMA_STEP))
    turn_noisy = turn + float(np.random.normal(0.0, SIGMA_TURN))

    rx += fwd_noisy * np.cos(rtheta)
    ry += fwd_noisy * np.sin(rtheta)
    rtheta += turn_noisy

    # Keep inside map bounds
    rx = np.clip(rx, 0.0, WIDTH - 1)
    ry = np.clip(ry, 0.0, HEIGHT - 1)

    return rx, ry, rtheta


def init_particles():
    particles = np.random.rand(NUM_PARTICLES, 3)
    particles *= np.array((WIDTH, HEIGHT, 2.0 * np.pi))
    return particles


def move_particles(particles, fwd, turn):
    if fwd == 0.0 and turn == 0.0:
        return particles
    particles[:, 0] += fwd * np.cos(particles[:, 2])
    particles[:, 1] += fwd * np.sin(particles[:, 2])
    particles[:, 2] += turn
    particles[:, 0] = np.clip(particles[:, 0], 0.0, WIDTH - 1)
    particles[:, 1] = np.clip(particles[:, 1], 0.0, HEIGHT - 1)
    return particles


# --- Sensor ---
def sense(x, y, noisy=False):
    
    SIGMA_SENSOR = 5.0
    xi = int(np.clip(x, 0, WIDTH - 1))
    yi = int(np.clip(y, 0, HEIGHT - 1))
    reading = float(map_blur[yi, xi])
    if noisy:
        reading += float(np.random.normal(0.0, SIGMA_SENSOR))
    return reading


# --- Weights ---
def compute_weights(particles, robot_sensor):
    
    # Vectorized
    xs = particles[:, 0].astype(np.int32)
    ys = particles[:, 1].astype(np.int32)
    particle_vals = map_blur[ys, xs].astype(np.float32)

    errors = np.abs(robot_sensor - particle_vals)
    # Convert error to weight (higher weight for smaller error)
    max_err = errors.max()
    if max_err == 0:
        weights = np.ones_like(errors)
    else:
        weights = (max_err - errors)

    # Kill off edge particles
    edge_mask = (
        (particles[:, 0] <= 0) |
        (particles[:, 0] >= WIDTH - 1) |
        (particles[:, 1] <= 0) |
        (particles[:, 1] >= HEIGHT - 1)
    )
    weights[edge_mask] = 0.0

    # Increase sensitivity
    weights = weights ** 3

    # Guard against all-zero
    if np.sum(weights) == 0:
        weights[:] = 1.0

    return weights


# --- Resample + jitter ---
def resample(particles, weights):
    probabilities = weights / np.sum(weights)
    idx = np.random.choice(NUM_PARTICLES, size=NUM_PARTICLES, p=probabilities)
    return particles[idx, :]


def add_noise(particles):
    
    SIGMA_PARTICLE_STEP = 2.0
    SIGMA_PARTICLE_TURN = np.pi / 24.0
    particles[:, 0] += np.random.normal(0, SIGMA_PARTICLE_STEP, NUM_PARTICLES)
    particles[:, 1] += np.random.normal(0, SIGMA_PARTICLE_STEP, NUM_PARTICLES)
    particles[:, 2] += np.random.normal(0, SIGMA_PARTICLE_TURN, NUM_PARTICLES)

    particles[:, 0] = np.clip(particles[:, 0], 0.0, WIDTH - 1)
    particles[:, 1] = np.clip(particles[:, 1], 0.0, HEIGHT - 1)
    return particles


# --- Display ---
def display(rx, ry, particles, show_particles=False, rtheta=0.0):
    lmap = cv2.cvtColor(map_blur, cv2.COLOR_GRAY2BGR)

    if show_particles:
        for i in range(NUM_PARTICLES):
            cv2.circle(lmap, (int(particles[i, 0]), int(particles[i, 1])), 1, (255, 0, 0), -1)

    # --- Draw robot heading arrow (green) ---
    pt1 = (int(rx), int(ry))
    arrow_len = 15
    pt2 = (
        int(rx + arrow_len * np.cos(rtheta)),
        int(ry + arrow_len * np.sin(rtheta))
    )
    cv2.arrowedLine(lmap, pt1, pt2, (0, 255, 0), 2, tipLength=0.4)

    # --- Draw best guess (red dot) ---
    if len(particles) > 0:
        px = float(np.mean(particles[:, 0]))
        py = float(np.mean(particles[:, 1]))
        cv2.circle(lmap, (int(px), int(py)), 5, (0, 0, 255), -1)

    # --- Display turn heading in degrees ---
    heading_deg = (np.degrees(rtheta) % 360)
    label = f"Heading: {heading_deg:.1f}°"
    text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
    pos = (WIDTH - text_size[0] - 10, text_size[1] + 10)
    cv2.putText(lmap, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Show the image
    cv2.imshow('map', lmap)



# --- Main loop ---
def main():
    global SHOW_PARTICLES
    particles = init_particles()

    cv2.namedWindow('map', cv2.WINDOW_AUTOSIZE)

    rx, ry, rtheta = (WIDTH / 4.0, HEIGHT / 4.0, 0.0)

    while True:
        display(rx, ry, particles, SHOW_PARTICLES, rtheta)
        fwd, turn, halt, toggle = get_input()

        if toggle:
            SHOW_PARTICLES = not SHOW_PARTICLES
            continue

        if halt:
            break

        # Update robot
        rx, ry, rtheta = move_robot(rx, ry, rtheta, fwd, turn)

        # Propagate particles (motion model)
        particles = move_particles(particles, fwd, turn)

        # Update & resample only if we moved forward (sensor update tied to new location)
        if fwd != 0.0:
            robot_sensor = sense(rx, ry, noisy=True)
            weights = compute_weights(particles, robot_sensor)
            particles = resample(particles, weights)
            particles = add_noise(particles)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
