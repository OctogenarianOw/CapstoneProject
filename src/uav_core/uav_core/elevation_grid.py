import numpy as np


class ElevationGrid:
    """Carte d'élévation 2.5D. Une cellule porte une altitude, sa
    variance et un compteur d'observations. Repère NED : z vers le bas.

    La variance ne descend jamais sous `min_sigma_m` au carré. Sans ce
    plancher, elle décroît en 1/n sans limite : après quelques centaines
    d'observations le gain tombe sous 0,002 et la cellule est figée sur
    sa valeur, qu'elle soit juste ou fausse. Aucune observation ultérieure,
    même prise de près et sous un meilleur angle, ne peut plus la corriger.
    """

    def __init__(self, size_m=100.0, resolution=1.0, origin_xy=(0.0, 0.0),
                 min_sigma_m=0.02):
        self.resolution = resolution
        self.n = int(size_m / resolution)
        self.origin_x = origin_xy[0] - size_m / 2.0
        self.origin_y = origin_xy[1] - size_m / 2.0

        # Incertitude minimale que la carte s'autorise à revendiquer.
        self.min_variance = float(min_sigma_m) ** 2

        self.elevation = np.full((self.n, self.n), np.nan, dtype=np.float32)
        self.variance = np.full((self.n, self.n), np.inf, dtype=np.float32)
        self.count = np.zeros((self.n, self.n), dtype=np.int32)

    def to_index(self, x, y):
        """Coordonnées monde vers indices de cellule. Tableaux numpy."""
        ix = ((x - self.origin_x) / self.resolution).astype(np.int32)
        iy = ((y - self.origin_y) / self.resolution).astype(np.int32)
        return ix, iy

    def update(self, points, variances):
        """points : tableau (N, 3) en NED.
        variances : tableau (N,) ou scalaire, en m^2."""
        if points.shape[0] == 0:
            return 0

        variances = np.asarray(variances, dtype=np.float32)
        if variances.ndim == 0:
            variances = np.full(points.shape[0], float(variances),
                                dtype=np.float32)

        ix, iy = self.to_index(points[:, 0], points[:, 1])

        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        ix, iy = ix[inside], iy[inside]
        z = points[inside, 2]
        r = variances[inside]
        if z.size == 0:
            return 0

        floor = self.min_variance

        for i, j, zm, rm in zip(ix, iy, z, r):
            if self.count[i, j] == 0:
                self.elevation[i, j] = zm
                self.variance[i, j] = max(rm, floor)
            else:
                p = self.variance[i, j]
                k = p / (p + rm)
                self.elevation[i, j] += k * (zm - self.elevation[i, j])
                self.variance[i, j] = max((1.0 - k) * p, floor)
            self.count[i, j] += 1

        return z.size

    def known_cells(self):
        return int(np.count_nonzero(self.count))