#!/usr/bin/python3
import numpy as np

def parse_expr(s):
    if isinstance(s, (int, float)):
        return float(s)
    s = s.replace(" ", "")
    s = s.replace("pi", str(np.pi))
    return eval(s, {"q": 0, "np": np})

def dh_transform_matrix(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])

def fk(dh_params, qs):
    T = np.eye(4)
    for i, (alpha_str, a, d, theta_str) in enumerate(dh_params):
        alpha = parse_expr(alpha_str)
        theta = parse_expr(theta_str) + qs[i]
        T = T @ dh_transform_matrix(a, alpha, d, theta)
    return T

dh1 = [
    ("pi/2", 0, 0.1025, "q"),       # J1
    ("pi/2", 0, 0, "pi + q"),       # J2
    ("pi/2", 0, 0.26, "pi/2 + q"),  # J3
    ("pi/2", 0, 0, "pi + q"),       # J4
    ("pi/2", 0, 0.264, "pi + q"),   # J5
    ("pi/2", 0, 0, "pi/2 + q"),     # J6
    ("0", 0, 0, "q"),           # J7
]
# old
dh2 = [
    ("-pi/2", 0, 0.1025, "q"),       # J1
    ("pi/2", 0, 0.0, "q"),           # J2
    ("-pi/2", 0, 0.26, "-pi/2 + q"), # J3
    ("pi/2", 0, 0.0, "q"),           # J4
    ("-pi/2", 0, 0.264, "q"),        # J5
    ("pi/2", 0, 0.0, "-pi/2 + q"),   # J6
    (0, 0, 0.0, "q"),                # J7
]

print("比较两组 DH 参数的末端位姿差异")
print("=" * 60)

for label, qs in [("全零位置", np.array([0, 0.1, 0, 0.1, 0, 0, 0])),
                  ("随机角度", np.random.uniform(-1.0, 1.0, 7))]:
    T1 = fk(dh1, qs)
    T2 = fk(dh2, qs)
    p1 = T1[:3, 3]
    p2 = T2[:3, 3]
    pos_diff = np.linalg.norm(p1 - p2)
    R1, R2 = T1[:3, :3], T2[:3, :3]
    R_diff = R1.T @ R2
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))

    print(f"\n{label} (q = {np.round(qs, 4).tolist()})")
    print(f"  DH1 位置: {np.round(p1, 6)}")
    print(f"  DH2 位置: {np.round(p2, 6)}")
    print(f"  位置差异: {pos_diff:.9f} m")
    print(f"  姿态差异: {np.degrees(angle):.6f} deg")
