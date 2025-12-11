import numpy as np
import xml.etree.ElementTree as ET

urdf_path = "/home/xc/dexe/dexe_robot_server/robot_server_lib/robot_models/W1Anthropomorphic/W1Anthropomorphic.urdf"

def skew(v):
    """返回向量的反对称矩阵"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def urdf_to_params(urdf_path, out_path="link_params.txt"):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    with open(out_path, "w") as f:
        link_id = 1
        for link in root.findall("link"):
            link_name = link.get("name")
            if not link_name.startswith("arm"):  # 只保留arm前缀的link
                continue

            inertial = link.find("inertial")
            if inertial is None:
                continue

            # mass
            mass_elem = inertial.find("mass")
            m = float(mass_elem.get("value")) if mass_elem is not None else 0.0

            # origin (center of mass)
            origin_elem = inertial.find("origin")
            if origin_elem is not None and "xyz" in origin_elem.attrib:
                com = np.array([float(x) for x in origin_elem.get("xyz").split()])
            else:
                com = np.zeros(3)

            # inertia (URDF 是 w.r.t COM)
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                I_com = np.array([
                    [float(inertia_elem.get("ixx", 0.0)), float(inertia_elem.get("ixy", 0.0)), float(inertia_elem.get("ixz", 0.0))],
                    [float(inertia_elem.get("ixy", 0.0)), float(inertia_elem.get("iyy", 0.0)), float(inertia_elem.get("iyz", 0.0))],
                    [float(inertia_elem.get("ixz", 0.0)), float(inertia_elem.get("iyz", 0.0)), float(inertia_elem.get("izz", 0.0))]
                ])
            else:
                I_com = np.zeros((3,3))

            # --- 平行轴定理修正: L = I_com + m * S(com)^T * S(com)
            S_com = skew(com)
            L = I_com + m * S_com.T @ S_com

            # --- 写入文件 ---
            f.write(f"L_{link_id}xx: {L[0,0]}\n")
            f.write(f"L_{link_id}xy: {L[0,1]}\n")
            f.write(f"L_{link_id}xz: {L[0,2]}\n")
            f.write(f"L_{link_id}yy: {L[1,1]}\n")
            f.write(f"L_{link_id}yz: {L[1,2]}\n")
            f.write(f"L_{link_id}zz: {L[2,2]}\n")
            f.write(f"l_{link_id}x: {m * com[0]}\n")
            f.write(f"l_{link_id}y: {m * com[1]}\n")
            f.write(f"l_{link_id}z: {m * com[2]}\n")
            f.write(f"m_{link_id}: {m}\n")

            link_id += 1

    print(f"参数已写入 {out_path}")


if __name__ == "__main__":
    urdf_to_params(urdf_path, "link_params.txt")
