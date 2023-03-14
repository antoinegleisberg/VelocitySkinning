#pragma once

#include "cgp/cgp.hpp"


namespace cgp
{
	struct rig_structure
	{
		numarray<numarray<int>> joint;
		numarray<numarray<float>> weight;
	};

	void normalize_weights(numarray<numarray<float>>& weights);

	void init_velocity_skinning_weights(
		rig_structure& velocity_rig,
		rig_structure const& rig,
		numarray<int> const& parent_index);

	void velocity_skinning_compute(
		numarray<vec3>& position_skinned,
		numarray<vec3>& normal_skinned,
		numarray<affine_rt> const& skeleton_current,
		numarray<affine_rt> const& skeleton_rest_pose,
		numarray<vec3> const& position_rest_pose,
		numarray<vec3> const& normal_rest_pose,
		rig_structure const& rig,
		rig_structure const& velocity_rig,
		numarray<affine_rt>& old_joint_rt,
		numarray<vec3>& old_velocity,
		float dt,
		float const speed_blending,
		float const linear_deformation_intensity,
		float const rotational_deformation_intensity
	);
	
	void compute_linear_velocity_deformation(
		int idx,
		numarray<vec3>& position_skinned,
		numarray<vec3> const& translation_velocity,
		rig_structure const& velocity_rig,
		numarray<vec3> const& old_velocity,
		float const speed_blending,
		float const deformation_intensity
	);

}