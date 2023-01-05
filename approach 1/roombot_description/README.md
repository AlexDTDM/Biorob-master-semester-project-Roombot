# Roombot description

## Total weight of one module

1.760 kg (see `rb_masses.pdf`)

## Computing inertial tags

Use `meshes/compute_inertia_tag.py` to compute automatically the URDF inertial tag from the .stl mesh (requirement: `pymeshlab`)

- based on https://github.com/vonunwerth/MeshLabInertiaToURDF , using center of mass instead of barycenter to compute the center of mass)
- computes the convex hull as first step (so no need to use Blender)
- script was probably inspired by https://classic.gazebosim.org/tutorials?cat=build_robot&tut=inertia


## Convex shapes

`convex` folder contains the exact convex shapes generated from actual size STLs in `non-scaled` subfolder 
