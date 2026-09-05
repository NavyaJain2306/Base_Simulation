import trimesh

# Load or generate your mesh
mesh = trimesh.load('2x2_base.obj')

# Force trimesh to compute the normals
_ = mesh.vertex_normals 

# Export the file and explicitly include normals
mesh.export('2x2_base.obj', include_normals=True)

print("Meshes Converted")
