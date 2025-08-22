import trimesh
import numpy as np
import os

def scale_model(file_path, output_path, scale_factor):
    """
    Scales a 3D model by a given factor and saves it to a new file.
    
    Args:
        file_path (str): Path to the input 3D model file.
        output_path (str): Path where the scaled model will be saved.
        scale_factor (float): Factor by which to scale the model.
    """
    # Load the 3D model
    mesh = trimesh.load(file_path)
    
    # Scale the mesh
    mesh.apply_scale(scale_factor)
    
    # Save the scaled mesh
    mesh.export(output_path)

if __name__ == "__main__":
    input_path = "/home/young/internal_repo/dexe_robot_client/dexe_robot_client/w1/path_planner/model/W1V02_1_collision"
    # link_names = ["arml1.obj", "arml2.obj", "arml3.obj", "arml4.obj", "arml5_short.obj", "arml6.obj", "arml7.obj",
    #               "armr1.obj", "armr2.obj", "armr3.obj", "armr4.obj", "armr5_short.obj", "armr6.obj", "armr7.obj",
    #                 "head1.obj", "head2.obj", "body1.obj", "body2.obj", "body3.obj", "body4.obj"]
    link_names = ["arml6.obj", "armr6.obj"]
    input_full_path = [os.path.join(input_path, name) for name in link_names]
    output_path = "/home/young/internal_repo/dexe_robot_client/dexe_robot_client/w1/path_planner/model/W1V02_1_collision/expand_obj"
    output_full_path = [os.path.join(output_path, name) for name in link_names]

    scale_factor = [1.1, 1.1, 1.1]
    for input_file, output_file in zip(input_full_path, output_full_path):
        scale_model(input_file, output_file, scale_factor)
        print(f"Scaled {input_file} and saved to {output_file}")