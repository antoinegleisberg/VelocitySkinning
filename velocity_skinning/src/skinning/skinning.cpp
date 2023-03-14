#include "skinning.hpp"

namespace cgp
{
	void normalize_weights(numarray<numarray<float>>& weights)
	{
		size_t const N = weights.size();
		for (size_t k = 0; k < N; ++k) {
			float s = 0.0f;
			for(float w : weights[k]) s += w;
			assert_cgp_no_msg(s>1e-5f);
			for(float& w : weights[k]) w /= s;
		}
	}


	/*
	These weights are used on the second version of the formula of velocity skinning:
		for vertex i, sum over descendent weights, and multiply by the speed of vertex i
	Second version (take regular weight of the joint, and sum over ascendent joints speed) use the regular weights
	*/
	void init_velocity_skinning_weights(
		rig_structure& velocity_rig,
		rig_structure const& rig,
		numarray<int> const& parent_index)
	{
		std::cout << "init_velocity_skinning_weights" << std::endl;

		int N_vertex = rig.joint.size();

		// convert the parent_index array into a descendence array
		// Note: the root is the only node with a parent_index of -1
		numarray<numarray<int>> descendents_index;
		descendents_index.resize(parent_index.size());
		for (int child = 0; child < parent_index.size(); child++) {
			int parent = child;
			while (parent != -1) {
				descendents_index[parent].push_back(child);
				parent = parent_index[parent];
			}
		}

		// compute the velocity weights
		velocity_rig.joint.resize(N_vertex);
		for (int i = 0; i < N_vertex; i++) {
			velocity_rig.joint[i].resize(rig.joint[i].size());
			for (int j = 0; j < rig.joint[i].size(); j++) {
				velocity_rig.joint[i][j] = rig.joint[i][j];
			}
		}

		velocity_rig.weight.resize(N_vertex);
		for (int i = 0; i < N_vertex; i++) {
			
			// calculate the weights associated to vertex i
			velocity_rig.weight[i].resize(rig.joint[i].size());
			
			for (int j = 0; j < rig.joint[i].size(); j++) {
				
				// for every joint to which vertex i is attached
				int joint = rig.joint[i][j];
				float velocity_weight = 0.0f;
				for (int k = 0; k < descendents_index[joint].size(); k++) {
					int child = descendents_index[joint][k];
					// int weight_index = rig.joint[i].index(child);
					int weight_index = 0;
					while (weight_index < rig.joint[i].size() && rig.joint[i][weight_index] != child) {
						weight_index++;
					}
					if (weight_index < rig.joint[i].size())
						velocity_weight += rig.weight[i][weight_index];
				}
				velocity_rig.weight[i][j] = velocity_weight;
			}
		}

		bool debug = false;
		
		if (debug) {
			for (int i = 0; i < descendents_index.size(); i++) {
				std::cout << "descendents of " << i << ": ";
				for (int j = 0; j < descendents_index[i].size(); j++) {
					std::cout << descendents_index[i][j] << " ";
				}
				std::cout << std::endl;
			}

			int v = 100;
			std::cout << "regular weights for vertex " << v << std::endl;
			for (int j = 0; j < rig.joint[v].size(); j++) {
				std::cout << "joint " << rig.joint[v][j] << ": " << rig.weight[v][j] << std::endl;
			}
			std::cout << "velocity weights for vertex " << v << std::endl;
			for (int j = 0; j < rig.joint[v].size(); j++) {
				std::cout << "joint " << rig.joint[v][j] << ": " << velocity_rig.weight[v][j] << std::endl;
			}
			
		}
	}
	
	
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
	)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		#pragma region LBS

		for (int i = 0; i < N_vertex; i++) {
			vec3 new_pos = vec3(0, 0, 0);
			vec3 new_normal = vec3(0, 0, 0);

			mat4 M = mat4::build_zero();

			for (int j = 0; j < rig.joint[i].size(); j++) {
				// compute the transformation matrix corresponding to the new position of that vertex
				int joint_nb = rig.joint[i][j];
				float weight = rig.weight[i][j];

				mat4 T = skeleton_current[joint_nb].matrix();
				mat4 T0inv = inverse(skeleton_rest_pose[joint_nb]).matrix();
				M += weight * T * T0inv;
			}

			position_skinned[i] = M * position_rest_pose[i];
			normal_skinned[i] = M * normal_rest_pose[i];
		}
		
		#pragma endregion

		#pragma region initialise velocity skinning variables

		// if old_joint_rt is empty, wait for next iteration
		if (old_joint_rt.size() == 0) {
			std::cout << "initialising old_joint_position" << std::endl;
			old_joint_rt.resize(N_joint);
			for (int i = 0; i < N_joint; i++) {
				old_joint_rt[i] = skeleton_current[i];
			}

			// also initialise old_velocity
			old_velocity.resize(N_joint);
			for (int i = 0; i < N_joint; i++) {
				old_velocity[i] = vec3(0, 0, 0);
			}
			return;
		}

		// if weights arent initialised yet, skip velocity skinning on first iteration
		if (velocity_rig.joint.size() == 0) {
			// std::cout << "velocity weights not initialised yet" << std::endl;
			return;
		}

		#pragma endregion
		
		#pragma region linear velocity skinning

		numarray<vec3> translation_velocity = numarray<vec3>(N_joint);
		for (int i = 0; i < N_joint; i++) {
			translation_velocity[i] = (skeleton_current[i].translation - old_joint_rt[i].translation) / dt;
		}

		// then, apply the deformation to all vertices
		for (int i = 0; i < N_vertex; i++) {
			compute_linear_velocity_deformation(
				i,
				position_skinned,
				translation_velocity,
				velocity_rig,
				old_velocity,
				speed_blending,
				linear_deformation_intensity
			);
		}

		#pragma endregion

		#pragma region rotational velocity skinning

		for (int i = 0; i < N_vertex; i++) {
			vec3 deformation = vec3(0, 0, 0);
			for (int j = 0; j < velocity_rig.joint[i].size(); j++) {
				int joint = velocity_rig.joint[i][j];
				affine_rt diff = skeleton_current[joint] * inverse(old_joint_rt[joint]);
				quaternion diff_q = diff.rotation.quat();

				float theta = 2 * std::atan2(norm(diff_q.xyz()), diff_q.w);
				if (norm(diff_q.xyz()) < 0.001) continue;
				vec3 angular_velocity_direction = normalize(diff_q.xyz());
				
				vec3 p_proj = skeleton_current[joint].translation + dot(position_skinned[i] - skeleton_current[joint].translation, angular_velocity_direction) * angular_velocity_direction;
				vec3 p_pi = position_skinned[i] - p_proj;
				// vec3 p_pi = position_skinned[i] - skeleton_current[joint].translation;
				float angle = norm(cross(angular_velocity_direction, p_pi) * theta) * 5;
				
				rotation_transform rotation = rotation_transform::from_axis_angle(angular_velocity_direction, angle);
				vec3 joint_j_deformation = rotation * (position_skinned[i] - skeleton_current[joint].translation) - 
					(position_skinned[i] - skeleton_current[joint].translation);
				deformation += joint_j_deformation * velocity_rig.weight[i][j];
			}
			position_skinned[i] -= deformation * rotational_deformation_intensity;
		}

		#pragma endregion
		
		#pragma region update velocity skinning variables

		// update old position & velocity
		for (int i = 0; i < N_joint; i++) {
			old_joint_rt[i] = skeleton_current[i];
			old_velocity[i] = (1 - speed_blending) * translation_velocity[i] + speed_blending * old_velocity[i];
		}

		#pragma endregion

	}

	
	void compute_linear_velocity_deformation(
		int idx,
		numarray<vec3>& position_skinned,
		numarray<vec3> const& translation_velocity,
		rig_structure const& velocity_rig,
		numarray<vec3> const& old_velocity,
		float const speed_blending,
		float const deformation_intensity
	)
	{
		vec3 deformation = vec3(0, 0, 0);
		for (int j = 0; j < velocity_rig.joint[idx].size(); j++) {
			int joint_nb = velocity_rig.joint[idx][j];
			float weight = velocity_rig.weight[idx][j];
			deformation += weight * ((1 - speed_blending) * translation_velocity[joint_nb] + speed_blending * old_velocity[joint_nb]);
		}
		
		position_skinned[idx] -= deformation * deformation_intensity;
	}
}