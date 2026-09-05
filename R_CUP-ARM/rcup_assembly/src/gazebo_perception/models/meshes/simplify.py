import trimesh

# Load your high-poly mesh
mesh = trimesh.load('4x2_base.obj')

# Method A: Specify absolute target face count via keyword argument
# simplified_mesh = mesh.simplify_quadric_decimation(target_count=200)

# Method B alternative: Specify reduction fraction (e.g., reduce by 80%)
simplified_mesh = mesh.simplify_quadric_decimation(0.8)

# Compute vertex normals so lighting renders properly in Gazebo
_ = simplified_mesh.vertex_normals

# Export the simplified mesh with normals included
simplified_mesh.export('4x2_base_simplified.obj', include_normals=True)
print(f"Original faces: {len(mesh.faces)}, New faces: {len(simplified_mesh.faces)}")
