import numpy as np
import os

GRID_SIZE = 200
TERRAIN_SIZE = 100.0
RESOLUTION = TERRAIN_SIZE / (GRID_SIZE - 1)
PLATEAU_HEIGHT = 3.5
ROUGHNESS_AMPLITUDE = 0.15
LANDING_PADS = [(0.60, 0.30, 0.04), (0.70, 0.70, 0.05), (0.55, 0.55, 0.03)]
ROUGH_ZONES = [(0.50, 0.50, 0.06, 0.8), (0.65, 0.15, 0.05, 0.6), (0.35, 0.40, 0.07, 0.5)]
BEACH_SLOPE_START = 0.2
BEACH_SLOPE_END = 0.45

def interp2d(small, target_size):
    h, w = small.shape
    x_in = np.linspace(0, w - 1, target_size)
    y_in = np.linspace(0, h - 1, target_size)
    xv, yv = np.meshgrid(x_in, y_in)
    x0 = np.floor(xv).astype(int)
    y0 = np.floor(yv).astype(int)
    x1 = np.minimum(x0 + 1, w - 1)
    y1 = np.minimum(y0 + 1, h - 1)
    fx = xv - x0
    fy = yv - y0
    return (small[y0, x0] * (1 - fx) * (1 - fy) +
            small[y0, x1] * fx * (1 - fy) +
            small[y1, x0] * (1 - fx) * fy +
            small[y1, x1] * fx * fy)

def generate():
    x = np.linspace(0, 1, GRID_SIZE)
    y = np.linspace(0, 1, GRID_SIZE)
    xv, yv = np.meshgrid(x, y)
    elev = np.zeros_like(xv)

    shore = xv < BEACH_SLOPE_START
    trans = (xv >= BEACH_SLOPE_START) & (xv < BEACH_SLOPE_END)
    plat = xv >= BEACH_SLOPE_END

    elev[shore] = -0.3 + (xv[shore] / BEACH_SLOPE_START) * 0.3
    t = (xv[trans] - BEACH_SLOPE_START) / (BEACH_SLOPE_END - BEACH_SLOPE_START)
    elev[trans] = (t * t * (3 - 2 * t)) * PLATEAU_HEIGHT
    elev[plat] = PLATEAU_HEIGHT

    elev += 0.4 * np.sin(2*np.pi*xv*1.5) * np.sin(2*np.pi*yv*1.2)
    elev += 0.2 * np.sin(2*np.pi*xv*3.0 + 0.5) * np.cos(2*np.pi*yv*2.5)

    np.random.seed(42)
    for freq in [8, 16, 32]:
        amp = ROUGHNESS_AMPLITUDE * (8.0 / freq)
        noise = np.random.randn(freq + 1, freq + 1)
        elev += amp * interp2d(noise, GRID_SIZE)

    for cx, cy, r, amp in ROUGH_ZONES:
        dist = np.sqrt((xv - cx)**2 + (yv - cy)**2)
        mask = dist < r
        elev[mask] += amp * np.sin(xv[mask]*80) * np.cos(yv[mask]*60)
        elev[mask] += amp * 0.5 * np.sin(xv[mask]*120 + yv[mask]*90)

    for cx, cy, r in LANDING_PADS:
        dist = np.sqrt((xv - cx)**2 + (yv - cy)**2)
        pad_h = PLATEAU_HEIGHT + 0.3*np.sin(2*np.pi*cx*1.5)*np.sin(2*np.pi*cy*1.2)
        blend = np.clip((r - dist) / (r * 0.3), 0, 1)
        elev = elev * (1 - blend) + pad_h * blend

    elev[shore] = np.minimum(elev[shore], 0.05)
    return elev

def export_obj(elev, path):
    rows, cols = elev.shape
    dx = TERRAIN_SIZE / (cols - 1)
    dy = TERRAIN_SIZE / (rows - 1)

    dz_dx = np.zeros_like(elev)
    dz_dy = np.zeros_like(elev)
    dz_dx[:, 1:-1] = (elev[:, 2:] - elev[:, :-2]) / (2*dx)
    dz_dx[:, 0] = (elev[:, 1] - elev[:, 0]) / dx
    dz_dx[:, -1] = (elev[:, -1] - elev[:, -2]) / dx
    dz_dy[1:-1, :] = (elev[2:, :] - elev[:-2, :]) / (2*dy)
    dz_dy[0, :] = (elev[1, :] - elev[0, :]) / dy
    dz_dy[-1, :] = (elev[-1, :] - elev[-2, :]) / dy
    norms = np.stack([-dz_dx, -dz_dy, np.ones_like(elev)], axis=2)
    norms /= np.maximum(np.linalg.norm(norms, axis=2, keepdims=True), 1e-8)

    with open(path, 'w') as f:
        f.write("mtllib coastal_terrain.mtl\nusemtl terrain_mat\n")
        for j in range(rows):
            for i in range(cols):
                px = (i/(cols-1))*TERRAIN_SIZE - TERRAIN_SIZE/2
                py = (j/(rows-1))*TERRAIN_SIZE - TERRAIN_SIZE/2
                f.write(f"v {px:.4f} {py:.4f} {elev[j,i]:.4f}\n")
        for j in range(rows):
            for i in range(cols):
                f.write(f"vt {i/(cols-1):.4f} {j/(rows-1):.4f}\n")
        for j in range(rows):
            for i in range(cols):
                n = norms[j,i]
                f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
        for j in range(rows-1):
            for i in range(cols-1):
                v00 = j*cols + i + 1
                v10 = v00 + 1
                v01 = v00 + cols
                v11 = v01 + 1
                f.write(f"f {v00}/{v00}/{v00} {v10}/{v10}/{v10} {v11}/{v11}/{v11}\n")
                f.write(f"f {v00}/{v00}/{v00} {v11}/{v11}/{v11} {v01}/{v01}/{v01}\n")
    print(f"OBJ: {rows*cols} vertices, {(rows-1)*(cols-1)*2} triangles")

if __name__ == "__main__":
    out = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/models/coastal_terrain")
    os.makedirs(out, exist_ok=True)

    print("Generating terrain...")
    elev = generate()
    print(f"Elevation: {elev.min():.2f} to {elev.max():.2f} m")

    export_obj(elev, os.path.join(out, "coastal_terrain.obj"))
    np.save(os.path.join(out, "elevation.npy"), elev)

    with open(os.path.join(out, "coastal_terrain.mtl"), 'w') as f:
        f.write("newmtl terrain_mat\nKa 0.3 0.3 0.3\nKd 0.6 0.55 0.4\nKs 0.0 0.0 0.0\nd 1.0\nillum 1\n")

    print("Done. Files in:", out)
    print("\nLanding pads (world coords):")
    for cx, cy, r in LANDING_PADS:
        wx = cx*TERRAIN_SIZE - TERRAIN_SIZE/2
        wy = cy*TERRAIN_SIZE - TERRAIN_SIZE/2
        gi, gj = int(cx*(GRID_SIZE-1)), int(cy*(GRID_SIZE-1))
        print(f"  ({wx:+.1f}, {wy:+.1f}, {elev[gj,gi]:.2f}) r={r*TERRAIN_SIZE:.0f}m")
