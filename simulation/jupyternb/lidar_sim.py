import numpy as np
from stl import mesh
import cmath
import copy


class LidarSumulator():

    def __init__(self, env_file, min_range=2.0, max_range=12000.0, resolution=1.):
        self.min_range = min_range
        self.max_range = max_range
        self.resolution = resolution  # in degrees
        self.load_env = self.load_environment(env_file)

    def load_environment(self, env_file):
        env = mesh.Mesh.from_file(env_file)
        env.x = env.x - env.x.min()
        env.y = env.y - env.y.min()
        return env

    def lidar_scann(self, x, y, yaw):
        triangles = self.get_env_triangles(x, y, yaw)
        return self.lidar_filter(triangles)

    def get_lidar_points(self, x, y, yaw):
        lidar_scann = self.lidar_scann(x, y, yaw)
        plot_scann = np.stack((np.arange(0, 2 * np.pi, np.radians(self.resolution)), lidar_scann), axis=1)
        return plot_scann[plot_scann[:, 1] != np.array(None)]

    def get_map_triangles(self):
        env = self.load_env
        subset = env.vectors[(env.normals[:, 0] == 0.0) & (env.normals[:, 1] == 0.0) & (env.normals[:, 2] < 0.0)]
        triangles = []
        for t in subset[:, :, :]:
            triangles.append(t[:,0:2])
        return np.array(triangles)

    def get_env_triangles(self, x, y, yaw):
        env = copy.deepcopy(self.load_env)
        env.x = env.x - x
        env.y = env.y - y
        env.rotate([0.0, 0.0, 1.0], yaw)
        subset = env.vectors[(env.normals[:, 0] == 0.0) & (env.normals[:, 1] == 0.0) & (env.normals[:, 2] < 0.0)]
        triangles = []
        for t in subset[:, :, :]:
            a = np.array(self._convert_array(t)).transpose()
            triangles.append(a[a[:, 0].argsort()])
        return np.array(triangles)

    def _cart2polC(self, xyz):
        x, y, z = xyz
        return (cmath.polar(complex(x, y)))  # rho, phi

    def _convert_array(self, arr):
        theta = []
        r = []
        for x in arr:
            rho, phi = self._cart2polC(x)
            theta.append(phi)
            r.append(rho)
        return theta, r

    def _get_distance(self, p1, p2, theta):
        theta_plus = theta
        if (p1[0] > np.pi) or (p2[0] > np.pi):
            # tringles are transformed in filter_triangles
            if theta < 0:
                theta_plus = theta + 2 * np.pi
        p = np.array([p1[0], p2[0]])
        if not (p.min() <= theta_plus) & (theta_plus <= p.max()):
            return self.max_range + 1.0
        r_s_1 = np.sin(p1[0]) * p1[1]
        r_c_1 = np.cos(p1[0]) * p1[1]
        r_s_2 = np.sin(p2[0]) * p2[1]
        r_c_2 = np.cos(p2[0]) * p2[1]
        m = (r_s_2 - r_s_1) / (r_c_2 - r_c_1)
        b = r_s_1 - m * r_c_1
        dist = b / (np.sin(theta_plus) - m * np.cos(theta_plus))
        return dist

    def _filter_triangles(self, triangles, theta):
        special_cases = triangles[np.any(triangles >= 0, axis=1)[:, 0] & np.any(triangles < 0, axis=1)[:, 0]]
        other_cases = triangles[np.invert(np.any(triangles >= 0, axis=1)[:, 0] & np.any(triangles < 0, axis=1)[:, 0])]
        # verticies on both sides
        triangles_hit = other_cases[np.any(other_cases >= theta, axis=1)[:, 0] & np.any(other_cases <= theta, axis=1)[:, 0]]
        # handle special cases:
        sc = []
        for t in special_cases:
            if ((t[:, 0].max() - t[:, 0].min()) < np.pi):
                # not so special after all
                if (theta <= t[:, 0].max()) & (t[:, 0].min() <= theta):
                    sc.append(t)
            else:
                for e in t:
                    if e[0] < 0:
                        e[0] += 2 * np.pi
                if (theta + 2 * np.pi <= t[:, 0].max()) & (t[:, 0].min() <= theta + 2 * np.pi):
                    sc.append(t)
                if (theta <= t[:, 0].max()) & (t[:, 0].min() <= theta):
                    sc.append(t)

        result = []
        for t in triangles_hit:
            result.append(t)
        for t in sc:
            result.append(t)
        return np.array(result)

    def lidar_filter(self, triangles):
        scann = []
        samples = np.arange(-np.pi, np.pi, np.radians(self.resolution))
        for sample in samples:
            # start with out of range
            dist = self.max_range + 1.0
            # select all triangles hit by the ray
            triangles_hit = self._filter_triangles(triangles, sample)
            for t in triangles_hit:
                dist_t = np.empty(3)
                dist_t[0] = self._get_distance(t[0], t[1], sample)
                dist_t[1] = self._get_distance(t[0], t[2], sample)
                dist_t[2] = self._get_distance(t[1], t[2], sample)
                dist = min(dist_t.min(), dist)
            scann.append(dist)
            if dist > self.max_range:
                scann[-1] = None
            if dist < self.min_range:
                scann[-1] = None
        return np.roll(np.array(scann), int(np.pi / np.radians(self.resolution)))


if __name__ == "__main__":
    test_lidar = LidarSumulator("square.stl")
    triangles = test_lidar.get_env_triangles(107, 189, 1.30899694)
    import pdb
    pdb.set_trace()
    triangles = test_lidar.get_map_triangles()
    print(test_lidar.lidar_scann(107, 189, 1.30899694))
