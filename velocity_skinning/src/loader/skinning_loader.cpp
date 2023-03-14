#include "skinning_loader.hpp"

using namespace cgp;

void load_animation_bend_z(numarray<numarray<affine_rt>>& animation_skeleton, numarray<float>& animation_time, numarray<int> const& )
{
	animation_time = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
	animation_skeleton.resize(animation_time.size());

	rotation_transform const r0 = rotation_transform();
	rotation_transform const rz1 = rotation_transform::from_axis_angle({0,0,1}, Pi/4.0f);
	rotation_transform const rz2 = rotation_transform::from_axis_angle({0,0,1}, Pi/2.0f);
	vec3 const t0 = {0.0f, 0, 0};
	vec3 const tx = {0.5f, 0, 0};
	animation_skeleton[0] = {affine_rt{r0,t0}, affine_rt{r0,tx}, affine_rt{r0,tx}};
	animation_skeleton[1] = {affine_rt{r0,t0}, affine_rt{rz1,tx}, affine_rt{r0,tx}};
	animation_skeleton[2] = {affine_rt{r0,t0}, affine_rt{rz2,tx}, affine_rt{r0,tx}};
	animation_skeleton[3] = animation_skeleton[1];
	animation_skeleton[4] = animation_skeleton[0];
}

void load_animation_bend_zx(numarray<numarray<affine_rt>>& animation_skeleton, numarray<float>& animation_time, numarray<int> const& )
{
	animation_time = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
	animation_skeleton.resize(animation_time.size());

	rotation_transform const r0 = rotation_transform();
	rotation_transform const rz = rotation_transform::from_axis_angle({0,0,1}, Pi/2.0f);
	rotation_transform const rx = rotation_transform::from_axis_angle({1,0,0}, Pi/2.0f);
	rotation_transform const rxn = rotation_transform::from_axis_angle({1,0,0}, -Pi/2.0f);
	vec3 const t0 = {0.0f, 0, 0};
	vec3 const tx = {0.5f, 0, 0};
	vec3 const tx2 = { 0, 0, 0 };
	animation_skeleton[0] = {affine_rt{r0,t0}, affine_rt{r0,tx}, affine_rt{r0,tx}};
	animation_skeleton[1] = {affine_rt{r0,t0}, affine_rt{rz,tx}, affine_rt{r0,tx}};
	animation_skeleton[2] = {affine_rt{r0,tx2}, affine_rt{rx,tx}*rz, affine_rt{r0,tx}};
	animation_skeleton[3] = {affine_rt{r0,t0}, affine_rt{rxn,tx}*rz, affine_rt{r0,tx}};
	animation_skeleton[4] = {affine_rt{r0,t0}, affine_rt{rz,tx}, affine_rt{r0,tx}};
	animation_skeleton[5] = animation_skeleton[0];

}

void load_animation_twist_x(numarray<numarray<affine_rt>>& animation_skeleton, numarray<float>& animation_time, numarray<int> const& )
{
	animation_time = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
	animation_skeleton.resize(animation_time.size());

	rotation_transform const r0 = rotation_transform();
	rotation_transform const rx1 = rotation_transform::from_axis_angle({1,0,0}, Pi/2.0f);
	rotation_transform const rx2 = rotation_transform::from_axis_angle({1,0,0}, Pi);
	vec3 const t0 = {0.0f, 0, 0};
	vec3 const tx = {0.5f, 0, 0};
	animation_skeleton[0] = {affine_rt{r0,t0}, affine_rt{r0,tx}, affine_rt{r0,tx}};
	animation_skeleton[1] = {affine_rt{r0,t0}, affine_rt{rx1,tx}, affine_rt{r0,tx}};
	animation_skeleton[2] = {affine_rt{r0,t0}, affine_rt{rx2,tx}, affine_rt{r0,tx}};
	animation_skeleton[3] = animation_skeleton[1];
	animation_skeleton[4] = animation_skeleton[0];
}


void load_animation_translation(numarray<numarray<affine_rt>>& animation_skeleton, numarray<float>& animation_time, numarray<int> const& )
{
	animation_time = { 0.0f, 1.0f, 2.0f };
	animation_skeleton.resize(animation_time.size());

	rotation_transform const r0 = rotation_transform();
	rotation_transform const r1 = rotation_transform::from_axis_angle({ 0,0,1 }, Pi / 4.0f);
	rotation_transform const r2 = rotation_transform::from_axis_angle({ 0,0,1 }, Pi / 2.0f);
	vec3 const t0 = { 0.0f, 0, 0 };
	vec3 const tx = { 0.5f, 0, 0 };
	vec3 const ty = { 0, 0.5f, 0 };
	animation_skeleton[0] = { affine_rt{r0,t0}, affine_rt{r0,tx}, affine_rt{r0,tx} };
	animation_skeleton[1] = { affine_rt{r0,t0}, affine_rt{r0,tx + ty}, affine_rt{r0,tx} };
	animation_skeleton[2] = { affine_rt{r0,t0}, affine_rt{r0,tx}, affine_rt{r0,tx} };
}

void load_cylinder(skeleton_animation_structure& skeleton_data, rig_structure& rig, mesh& shape)
{
	// Skeleton
	skeleton_data.parent_index = {-1, 0, 1};
	
	rotation_transform r0 = rotation_transform();
	skeleton_data.rest_pose_local.resize(3);
	skeleton_data.rest_pose_local[0] = affine_rt( r0, vec3{0.0f,0,0} );
	skeleton_data.rest_pose_local[1] = affine_rt( r0, vec3{0.5f,0,0} );
	skeleton_data.rest_pose_local[2] = affine_rt( r0, vec3{0.5f,0,0} );

	// mesh
	size_t const N = 50;
	shape = mesh_primitive_cylinder(0.1f, {0,0,0}, {1,0,0}, N,N);
	
	rig.joint.clear();
	rig.weight.clear();

	for (size_t ku = 0; ku < N; ++ku)
	{
		for (size_t kv = 0; kv < N; ++kv)
		{
			const float u = ku/float(N-1.0f);
			float const alpha = 3.0f; // power for skinning weights evolution
			float w0, w1;

			if (u < 0.5f) {
				w1 = 0.5f*std::pow(u/0.5f, alpha);
				w0 = 1-w1;
			}
			else {
				w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
				w1 = 1-w0;
			}
			
			rig.joint.push_back(numarray<int>{0, 1});
			rig.weight.push_back(numarray<float>{w0, w1});
		}
	}
}

void load_rectangle(skeleton_animation_structure& skeleton_data, rig_structure& rig, mesh& shape)
{
	// Skeleton
	skeleton_data.parent_index = {-1, 0, 1};
	
	rotation_transform r0 = rotation_transform();
	skeleton_data.rest_pose_local.resize(3);
	skeleton_data.rest_pose_local[0] = affine_rt( r0, vec3{0.0f,0,0} );
	skeleton_data.rest_pose_local[1] = affine_rt( r0, vec3{0.5f,0,0} );
	skeleton_data.rest_pose_local[2] = affine_rt( r0, vec3{0.5f,0,0} );

	// mesh
	float const a = 0.1f;
	shape = mesh_primitive_cubic_grid({0,-a,-a}, {1,-a,-a}, {1,a,-a}, {0,a,-a}, {0,-a,a}, {1,-a,a}, {1,a,a}, {0,a,a}, 35, 10, 10);
	
	rig.joint.clear();
	rig.weight.clear();

	size_t const N = shape.position.size();
	for(size_t k=0; k<N; ++k)
	{
		float const u = shape.position[k].x;
		float const alpha = 3.0f;
		float w0, w1;

		if (u < 0.5f) {
			w1 = 0.5f*std::pow(u/0.5f, alpha);
			w0 = 1-w1;
		}
		else {
			w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
			w1 = 1-w0;
		}
		rig.joint.push_back(numarray<int>{0, 1});
		rig.weight.push_back(numarray<float>{w0, w1});
	}

}


//Map correspondance between skinning weights and vertices (that have been duplicated to load the texture coordinates)
template <typename T>
numarray<T> map_correspondance(numarray<T> value, numarray<numarray<int> > const& correspondance)
{
    // find the maximal index used for destination
    int max_index = 0;
	for (int k1 = 0; k1 < correspondance.size(); ++k1) {
		for (int k2 = 0; k2 < correspondance[k1].size(); ++k2) {
			if (max_index < correspondance[k1][k2]) {
				max_index = correspondance[k1][k2];
			}
		}
	}

	numarray<T> new_value;
    new_value.resize(max_index+1);

    // Apply correspondance (copy value[k] in its destination)
	for (int k = 0; k < correspondance.size(); ++k) {
		for (int k2 = 0; k2 < correspondance[k].size(); ++k2) {
			new_value[correspondance[k][k2]] = value[k];
		}
	}
    return new_value;
}