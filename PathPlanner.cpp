#include "PathPlanner.h"

PathPlanner::PathPlanner() :
	octree_(0.01f),
	num_nodes_(5000),
	ref_p_nn_(20),
	prmcegraph_(num_nodes_),
	num_joints_(6),
	//rand_gen_(time(0))
	rand_gen_(1),
	arm_radius_(0.07f),
	num_ref_points_(7),
	prmce_round_counter_(1),
	prmce_swept_volume_counter_(1),
	prmce_collision_found_(false),
	path_planner_ready_(false)
{
	// DH parameters
	a_[0] = a_[3] = a_[4] = a_[5] = 0.f; a_[1] = -0.612f; a_[2] = -0.5732f;
	d_[1] = d_[2] = 0.; d_[0] = 0.1273; d_[3] = 0.163941; d_[4] = 0.1157; d_[5] = 0.0922; //UR
	//d_[1] = 0.05f; d_[2] = 0.f; d_[0] = 0.1273f; d_[3] = 0.113941f; d_[4] = 0.1157f; d_[5] = 0.0922f; //Dylan
	alpha_[1] = alpha_[2] = alpha_[5] = 0.f; alpha_[0] = alpha_[3] = 1.570796327f; alpha_[4] = -1.570796327f;
}

PathPlanner::~PathPlanner()
{
	if(random_nodes_buffer_ != NULL) delete[] random_nodes_buffer_;
	if(reference_points_buffer_ != NULL) delete[] reference_points_buffer_;
	if(start_end_ref_points_ != NULL) delete[] start_end_ref_points_;
}

/*
	false: no collision, this is old
*/
bool PathPlanner::collisionCheck(float* joint_array6, float radius)
{
	forwardKinematicsUR10(joint_array6);

	arm_joint_points_.clear();

	// calculate points on the 6 joints 
	Eigen::Matrix4f mat;
	
	mat = DH_mat_vec_[0];

	Eigen::Vector4f eigen_point(0.0f, 0.0f, 0.17f, 1.0f);
	eigen_point = mat*eigen_point;
	PointT p; p.x = eigen_point(0); p.y = eigen_point(1); p.z = eigen_point(2);

	//PointT p; p.x = mat(0, 3); p.y = mat(1, 3); p.z = mat(2, 3);
	
	arm_joint_points_.push_back(p);

	mat = mat * DH_mat_vec_[1];

	p.x = mat(0, 3); p.y = mat(1, 3); p.z = mat(2, 3);

	arm_joint_points_.push_back(p);

	for (int i = 2; i < 6; i++)
	{
		// DH0*DH1*DH2...*DHn
		mat = mat * DH_mat_vec_[i];

		PointT p1; p1.x = mat(0, 3); p1.y = mat(1, 3); p1.z = mat(2, 3);

		arm_joint_points_.push_back(p1);

		//std::cout << arm_joint_points_.back() << "\n";
	}

	// add kinect end points
	Eigen::Vector4f point_kinect(0.0540247, 0.1026325, 0.0825227, 1.);
	point_kinect = mat*point_kinect;

	p.x = point_kinect(0);
	p.y = point_kinect(1);
	p.z = point_kinect(2);
	arm_joint_points_.push_back(p);

	// add points on the two long arm segments
	int num_inter_points = 5;
	for (int j = 0; j < 2; j++)
	for (int i = 1; i <= num_inter_points; i++)
	{
		PointT p2;

		float alpha = (float)i*(1.0f / (num_inter_points + 1));

		p2.x = (1.0f - alpha)*arm_joint_points_[j].x + alpha*arm_joint_points_[j+1].x;
		p2.y = (1.0f - alpha)*arm_joint_points_[j].y + alpha*arm_joint_points_[j + 1].y;
		p2.z = (1.0f - alpha)*arm_joint_points_[j].z + alpha*arm_joint_points_[j + 1].z;

		arm_joint_points_.push_back(p2);
	}

	// knn search k=1
	for (auto p : arm_joint_points_)
	{
		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;

		if (octree_.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (sqrt(pointNKNSquaredDistance[0]) < radius)
			{
				//std::cout << "collision found at " << p << "\n";
				return true;
			}
		}
	}

	return false;
}


/*
	http://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/actual-center-of-mass-for-robot-17264/
	https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
*/
Eigen::Matrix4f PathPlanner::constructDHMatrix(int target_joint_id, float target_joint_pos)
{
	Eigen::Matrix4f DH_mat;
	DH_mat = Eigen::Matrix4f::Identity();
	float cos_target = cos(target_joint_pos);
	float sin_target = sin(target_joint_pos);
	float cos_alp_tar = cos(alpha_[target_joint_id]);
	float sin_alp_tar = sin(alpha_[target_joint_id]);

	DH_mat(0, 0) = cos_target;
	DH_mat(0, 1) = -sin_target*cos_alp_tar;
	DH_mat(0, 2) = sin_target*sin_alp_tar;
	DH_mat(0, 3) = a_[target_joint_id] * cos_target;
	DH_mat(1, 0) = sin_target;
	DH_mat(1, 1) = cos_target*cos_alp_tar;
	DH_mat(1, 2) = -cos_target*sin_alp_tar;
	DH_mat(1, 3) = a_[target_joint_id] * sin_target;
	DH_mat(2, 1) = sin_alp_tar;
	DH_mat(2, 2) = cos_alp_tar;
	DH_mat(2, 3) = d_[target_joint_id];

	/*DH_mat(0, 0) = cos(target_joint_pos);
	DH_mat(0, 1) = -sin(target_joint_pos)*cos(alpha_[target_joint_id]);
	DH_mat(0, 2) = sin(target_joint_pos)*sin(alpha_[target_joint_id]);
	DH_mat(0, 3) = a_[target_joint_id]*cos(target_joint_pos);
	DH_mat(1, 0) = sin(target_joint_pos);
	DH_mat(1, 1) = cos(target_joint_pos)*cos(alpha_[target_joint_id]);
	DH_mat(1, 2) = -cos(target_joint_pos)*sin(alpha_[target_joint_id]);
	DH_mat(1, 3) = a_[target_joint_id]*sin(target_joint_pos);
	DH_mat(2, 1) = sin(alpha_[target_joint_id]);
	DH_mat(2, 2) = cos(alpha_[target_joint_id]);
	DH_mat(2, 3) = d_[target_joint_id];
*/
	return DH_mat;
}

void PathPlanner::forwardKinematicsUR10(float* joint_array6)
{
	for (int i = 0; i < 6; i++)	DH_mat_vec_[i] = constructDHMatrix(i, joint_array6[i]);
	fk_mat_ = DH_mat_vec_[0];
	for (int i = 1; i < 6; i++) fk_mat_ = fk_mat_*DH_mat_vec_[i];
}

/*
	center_config/neighbor_config: start pointer to an array (6 joint pos values)
	step_size: in rad in Cspace
*/
bool PathPlanner::checkCollisionBetweenTwoConfig(float* center_config, float* neighbor_config, float dist, float step_size)
{
	float inter_joint_pos[6];

	// normalize
	float alpha_step = step_size / dist;

	// bisection style
	for (float layer_alpha = 0.5f; layer_alpha >= alpha_step; layer_alpha *= 0.5f)
	{
		for (float alpha = layer_alpha; alpha < 1.0f; alpha += 2.f*layer_alpha)
		{
			for (int i = 0; i < 6; i++)	inter_joint_pos[i] = (alpha*neighbor_config[i] + (1.0f - alpha)*center_config[i]);

			// collision check
			if (collisionCheck(inter_joint_pos, 0.07f)) return true;
		}
	}

	// no collision
	return false;
}

// reset the grid, origin of the 3d grid is at bottom left corner
int PathPlanner::initGrid(int width, int depth, int height, int cell_size, int offset_x, int offset_y, int offset_z)
{
	if (width % cell_size != 0 || depth % cell_size != 0 || height % cell_size != 0 ||
		offset_x % cell_size != 0 || offset_y % cell_size != 0 || offset_z % cell_size != 0)
	{
		std::cout << "Found a dimension not divisible by cell size\n";
		return -1;
	}

	grid_width_cm_ = width; grid_depth_cm_ = depth; grid_height_cm_ = height;
	cell_size_ = cell_size;
	grid_width_ = grid_width_cm_ / cell_size_;
	grid_depth_ = grid_depth_cm_ / cell_size_;
	grid_height_ = grid_height_cm_ / cell_size_;
	grid_offset_x_ = offset_x; grid_offset_y_ = offset_y; grid_offset_z_ = offset_z;
	num_cells_ = grid_width_*grid_depth_*grid_height_;

	grid_.clear(); grid_.resize(num_cells_);

	for (auto cell : grid_)
	{
		cell.isOccupiedCounter = 0;
		cell.sweptVolumneCounter = 0;
	}

	return 0;
}

/*
	float* is the pointer to coordinate x
*/
void PathPlanner::blockCells(PointT & point_in_arm_base)
{
		// convert to cm and floor
		int x = ((int)(point_in_arm_base.x*100.f) - grid_offset_x_) / cell_size_;
		int y = ((int)(point_in_arm_base.y*100.f) - grid_offset_y_) / cell_size_;
		int z = ((int)(point_in_arm_base.z*100.f) - grid_offset_z_) / cell_size_;

		// check out of boundary
		if (x >= grid_width_ || x<0 || y >= grid_depth_ || y<0 || z >= grid_height_ || z<0)
		{
			//std::cout << "out of bound" << point_in_arm_base << "\n";
			return;
		}

		int cell_idx = x + y*grid_width_ + z*grid_width_*grid_depth_;

		if (grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
		{
			grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
				
			// change associated edge weight to Inf
			for (auto e : grid_[cell_idx].edges)
			{
				std::pair<prmceedge_descriptor, bool> eb = boost::edge(e.first, e.second, prmcegraph_);
				if(eb.second) prmcegraph_[eb.first].weight = 1000.f;// std::numeric_limits<float>::max();
			}
		}
}

bool PathPlanner::addEdgeDescriptor2Cell(std::vector<prmceedge_descriptor> & edge_vec, int cell_index)
{
	//for (auto e : grid_[cell_index].edges)
	//{
	//	// check if edge is already in the list
	//	if (e.m_source == edge.m_source && e.m_target == edge.m_target)	return false;
	//}
	//grid_[cell_index].edges.push_back(edge);
	
	for (auto e : edge_vec)
	{
		std::pair<int, int> edge_pair(e.m_source, e.m_target);
		grid_[cell_index].edges.push_back(edge_pair);
	}

	return true;
}


/*
	rot_mats, UR10 DH frames 1 to 6
*/
void PathPlanner::computeReferencePointsOnArm(float* joint_pos, std::vector<RefPoint> & reference_points, std::vector<Eigen::Matrix3f> & rot_mats)
{
	// forward kinematics 
	Eigen::Matrix4f dh_mat; dh_mat = constructDHMatrix(0, joint_pos[0]);

	// accumulated dh matrix
	Eigen::Matrix4f accum_dh_mat(dh_mat); Eigen::Vector4f vec; RefPoint ref_p;

	// first cross length (center to center) 18 cm, second 13cm

	reference_points.clear();
	rot_mats.clear();

	// this is for rover base
	//rot_mats.push_back(Eigen::Matrix3f::Identity());

	// origin
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = 0.f; 
	reference_points.push_back(ref_p);

	// frame 1 on ur10 dh figure
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = accum_dh_mat(i, 3);  
	reference_points.push_back(ref_p);
	rot_mats.push_back(accum_dh_mat.block<3,3>(0,0));

	// frame 1, the other side, z positive
	vec(0) = 0.f; vec(1) = 0.f; vec(2) = 0.18f, vec(3) = 1.f; vec = accum_dh_mat * vec;
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = vec(i); 
	reference_points.push_back(ref_p);

	// frame 2, the other side, z positive
	dh_mat = constructDHMatrix(1, joint_pos[1]); accum_dh_mat = accum_dh_mat * dh_mat;
	vec(0) = 0.f; vec(1) = 0.f; vec(2) = 0.18f, vec(3) = 1.f; vec = accum_dh_mat * vec;
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = vec(i);  
	reference_points.push_back(ref_p);
	rot_mats.push_back(accum_dh_mat.block<3, 3>(0, 0));

	// frame 2, near the frame 2, z positive
	vec(0) = 0.f; vec(1) = 0.f; vec(2) = 0.05f, vec(3) = 1.f; vec = accum_dh_mat * vec;
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = vec(i); 
	reference_points.push_back(ref_p);

	// frame 3
	dh_mat = constructDHMatrix(2, joint_pos[2]); accum_dh_mat = accum_dh_mat * dh_mat;
	vec(0) = 0.f; vec(1) = 0.f; vec(2) = 0.05f, vec(3) = 1.f; vec = accum_dh_mat * vec;
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = vec(i);
	reference_points.push_back(ref_p);
	rot_mats.push_back(accum_dh_mat.block<3, 3>(0, 0));

	// frame 4, 5, 6
	for (int i = 3; i < 6; i++)
	{
		dh_mat = constructDHMatrix(i, joint_pos[i]); accum_dh_mat = accum_dh_mat * dh_mat;
		for (int j = 0; j < 3; j++) ref_p.coordinates[j] = accum_dh_mat(j, 3); 
		reference_points.push_back(ref_p);
		rot_mats.push_back(accum_dh_mat.block<3, 3>(0, 0));
	}
}


/*
	http://www.geometrictools.com/Documentation/DynamicCollisionDetection.pdf
	separating axis theorem
	D = C1 - C0
	C(i,j) = A(i)*B(j)
*/
bool PathPlanner::collisionOBB(OBB & obb0, OBB & obb1)
{
	float R0, R1, R;
	Eigen::Vector3f a, b, L, D, AD, BD;
	Eigen::Matrix3f A, B, C, absC;

	D = obb1.C - obb0.C;
	a = obb0.a; 
	b = obb1.a;
	A = obb0.A;
	B = obb1.A;
	AD = A.transpose()*D;
	BD = B.transpose()*D;
	C = A.transpose()*B;
	absC = C.cwiseAbs();
	
	// 1: L = A0; 
	R0 = a(0); 
	R1 = b(0)*absC(0,0) + b(1)*absC(0,1) + b(2)*absC(0,2);
	R = abs(AD(0));
	if (R > R0 + R1) return false;

	// 2: L = A1;
	R0 = a(1);
	R1 = b(0)*absC(1, 0) + b(1)*absC(1, 1) + b(2)*absC(1, 2);
	R = abs(AD(1));
	if (R > R0 + R1) return false;
	
	// 3: L = A2;
	R0 = a(2);
	R1 = b(0)*absC(2, 0) + b(1)*absC(2, 1) + b(2)*absC(2, 2);
	R = abs(AD(2));
	if (R > R0 + R1) return false;

	// 4: L = B0;
	R0 = a(0)*absC(0, 0) + a(1)*absC(1, 0) + a(2)*absC(2, 0);
	R1 = b(0);
	R = abs(BD(0));
	if (R > R0 + R1) return false;

	// 5: L = B1;
	R0 = a(0)*absC(0, 1) + a(1)*absC(1, 1) + a(2)*absC(2, 1);
	R1 = b(1);
	R = abs(BD(1));
	if (R > R0 + R1) return false;

	// 6: L = B2;
	R0 = a(0)*absC(0, 2) + a(1)*absC(1, 2) + a(2)*absC(2, 2);
	R1 = b(2);
	R = abs(BD(2));
	if (R > R0 + R1) return false;

	// 7: L = A0 x B0;
	R0 = a(1)*absC(2, 0) + a(2)*absC(1, 0);
	R1 = b(1)*absC(0, 2) + b(2)*absC(0, 1);
	R = abs(C(1, 0)*AD(2) - C(2, 0)*AD(1));
	if (R > R0 + R1) return false;

	// 8: L = A0 x B1;
	R0 = a(1)*absC(2, 1) + a(2)*absC(1, 1);
	R1 = b(0)*absC(0, 2) + b(2)*absC(0, 0);
	R = abs(C(1, 1)*AD(2) - C(2, 1)*AD(1));
	if (R > R0 + R1) return false;

	// 9: L = A0 x B2;
	R0 = a(1)*absC(2, 2) + a(2)*absC(1, 2);
	R1 = b(0)*absC(0, 1) + b(1)*absC(0, 0);
	R = abs(C(1, 2)*AD(2) - C(2, 2)*AD(1));
	if (R > R0 + R1) return false;

	// 10: L = A1 x B0;
	R0 = a(0)*absC(2, 0) + a(2)*absC(0, 0);
	R1 = b(1)*absC(1, 2) + b(2)*absC(1, 1);
	R = abs(C(2, 0)*AD(0) - C(0, 0)*AD(2));
	if (R > R0 + R1) return false;

	// 11: L = A1 x B1;
	R0 = a(0)*absC(2, 1) + a(2)*absC(0, 1);
	R1 = b(0)*absC(1, 2) + b(2)*absC(1, 0);
	R = abs(C(2, 1)*AD(0) - C(0, 1)*AD(2));
	if (R > R0 + R1) return false;

	// 12: L = A1 x B2;
	R0 = a(0)*absC(2, 2) + a(2)*absC(0, 2);
	R1 = b(0)*absC(1, 1) + b(1)*absC(1, 0);
	R = abs(C(2, 2)*AD(0) - C(0, 2)*AD(2));
	if (R > R0 + R1) return false;

	// 13: L = A2 x B0;
	R0 = a(0)*absC(1, 0) + a(1)*absC(0, 0);
	R1 = b(1)*absC(2, 2) + b(2)*absC(2, 1);
	R = abs(C(0, 0)*AD(1) - C(1, 0)*AD(0));
	if (R > R0 + R1) return false;

	// 14: L = A2 x B1;
	R0 = a(0)*absC(1, 1) + a(1)*absC(0, 1);
	R1 = b(0)*absC(2, 2) + b(2)*absC(2, 0);
	R = abs(C(0, 1)*AD(1) - C(1, 1)*AD(0));
	if (R > R0 + R1) return false;

	// 15: L = A2 x B2;
	R0 = a(0)*absC(1, 2) + a(1)*absC(0, 2);
	R1 = b(0)*absC(2, 1) + b(1)*absC(2, 0);
	R = abs(C(0, 2)*AD(1) - C(1, 2)*AD(0));
	if (R > R0 + R1) return false;

	return true;
}

/*
	p0, p1 are end points on the arm line segment, 
	rot is rotation matrix
	axis: align with p0-p1 line segment x-0 y-1 z-2
*/
void PathPlanner::constructOBB(RefPoint & p0, RefPoint & p1, Eigen::Matrix3f rot, float radius, int axis, OBB & obb)
{
	Eigen::Vector3f point0(p0.coordinates[0], p0.coordinates[1], p0.coordinates[2]);
	Eigen::Vector3f point1(p1.coordinates[0], p1.coordinates[1], p1.coordinates[2]);

	// center
	obb.C = 0.5f*(point0 + point1);
	
	obb.A = rot;

	// extents
	obb.a << radius, radius, radius;
	obb.a(axis) = (point0 - point1).norm()*0.5f;
}

void PathPlanner::getArmOBBModel(std::vector<RefPoint> ref_points, std::vector<Eigen::Matrix3f> & rot_mats, std::vector<OBB> & arm_obbs)
{
	arm_obbs.clear();

	RefPoint tmp_rp, tmp_rp1;

	// add rover base
	OBB obb;
	obb.C << 0.f, 0.2f, -0.48f;
	obb.A = Eigen::Matrix3f::Identity();
	obb.a << 0.37f, 0.38f, 0.47f;
	arm_obbs.push_back(obb);

	// add chamber window top wall, robot arm base center to front edge 16.5cm, robot-to-window 20cm, wall thickness 5cm
	// floor-to-base height 0.964m, 
	obb.C << 0.f, -0.39f, 1.161f;
	obb.a << 1.3f, 0.025f, 0.155f;
	arm_obbs.push_back(obb);

	// add chamber inside wall
	obb.C << 0.f, -1.19f, 0.17f;
	obb.a << 1.3f, 0.025f, 1.4f;
	arm_obbs.push_back(obb);

	// add chamber table
	obb.C << 0.f, -0.7f, -0.732f;
	obb.a << 1.3f, 0.5f, 0.01f;
	arm_obbs.push_back(obb);

	// frame 1 arm, UR10 DH figure, y axis
	ref_points[1].coordinates[2] = 0.21f;
	constructOBB(ref_points[0], ref_points[1], rot_mats[0], 0.11f, 1, obb);
	arm_obbs.push_back(obb);

	// 1st long arm, frame 2
	for (int i = 0; i < 3; i++) ref_points[2].coordinates[i] += rot_mats[1](i, 0)*0.08f;
	for (int i = 0; i < 3; i++) ref_points[3].coordinates[i] -= rot_mats[1](i, 0)*0.06f;
	constructOBB(ref_points[2], ref_points[3], rot_mats[1], 0.069f, 0, obb);
	arm_obbs.push_back(obb);

	// 2nd long arm, frame 3
	for (int i = 0; i < 3; i++) ref_points[4].coordinates[i] += rot_mats[2](i, 0)*0.06f;
	for (int i = 0; i < 3; i++) ref_points[5].coordinates[i] -= rot_mats[2](i, 0)*0.046f;
	constructOBB(ref_points[4], ref_points[5], rot_mats[2], 0.045f, 0, obb);
	arm_obbs.push_back(obb);

	// frame 4
	for (int i = 0; i < 3; i++) ref_points[6].coordinates[i] -= rot_mats[3](i, 2)*0.06f;
	
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[7].coordinates[i] - rot_mats[3](i, 2)*0.051f;
	constructOBB(ref_points[6], tmp_rp, rot_mats[3], 0.052f, 2, obb);
	arm_obbs.push_back(obb);

	// frame 5
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[7].coordinates[i] - rot_mats[4](i, 2)*0.045f;
	constructOBB(tmp_rp, ref_points[8], rot_mats[4], 0.045f, 2, obb);
	arm_obbs.push_back(obb);

	// sensor block (kinect)
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.14f + rot_mats[5](i, 2)*0.041f + rot_mats[5](i, 1)*0.1f;
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] - rot_mats[5](i, 0)*0.14f + rot_mats[5](i, 2)*0.041f + rot_mats[5](i, 1)*0.1f;
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.04f, 0, obb);
	arm_obbs.push_back(obb);

	// laser scanner
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.09f + rot_mats[5](i, 2)*0.05f + rot_mats[5](i, 1)*0.01f;
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] - rot_mats[5](i, 0)*0.09f + rot_mats[5](i, 2)*0.05f + rot_mats[5](i, 1)*0.01f;
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.045f, 0, obb);
	arm_obbs.push_back(obb);

	// probe stick	(0.035425, -0.0445422, 0.184104)
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.035425f + rot_mats[5](i, 1)*(-0.0445422f) + rot_mats[5](i, 2)*0.005f;
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.035425f + rot_mats[5](i, 1)*(-0.0445422f) + rot_mats[5](i, 2)*0.184104f;
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.005f, 2, obb);
	arm_obbs.push_back(obb);

}

bool PathPlanner::selfCollision(float* joint_pos)
{
	std::vector<RefPoint> reference_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<OBB> arm_obbs;

	computeReferencePointsOnArm(joint_pos, reference_points, rot_mats);

	if (reference_points.back().coordinates[1] > 0.3f) return true;

	getArmOBBModel(reference_points, rot_mats, arm_obbs);

	for (int i = start_check_obb_idx_; i < arm_obbs.size(); i++)
	{
		for (int j = 0; j < i; j++)
		{
			if (collisionOBB(arm_obbs[i], arm_obbs[j]))
			{
				//std::cout << i <<" collide with " << j << "\n";
				return true;
			}
		}
	}
	return false;
}

int PathPlanner::voxelizeLine(RefPoint & p1, RefPoint & p2, std::vector<RefPoint> & saved_points_grid_frame, std::vector<prmceedge_descriptor> & edge_vec,
								bool point_in_robot_base_frame=true, bool save_points=false, bool set_occupy=false, bool set_swept_volume=false)
{
	int x1, y1, z1, x2, y2, z2;
	int num_voxelized = 0;
	
	if (point_in_robot_base_frame)
	{
		// convert to cm and floor, to grid coordinates
		x1 = ((int)(p1.coordinates[0] * 100.f) - grid_offset_x_) / cell_size_;
		y1 = ((int)(p1.coordinates[1] * 100.f) - grid_offset_y_) / cell_size_;
		z1 = ((int)(p1.coordinates[2] * 100.f) - grid_offset_z_) / cell_size_;

		x2 = ((int)(p2.coordinates[0] * 100.f) - grid_offset_x_) / cell_size_;
		y2 = ((int)(p2.coordinates[1] * 100.f) - grid_offset_y_) / cell_size_;
		z2 = ((int)(p2.coordinates[2] * 100.f) - grid_offset_z_) / cell_size_;
	}
	else
	{
		// should be in cm already
		x1 = (int)p1.coordinates[0];
		y1 = (int)p1.coordinates[1];
		z1 = (int)p1.coordinates[2];

		x2 = (int)p2.coordinates[0];
		y2 = (int)p2.coordinates[1];
		z2 = (int)p2.coordinates[2];
	}

	int dx = x2 - x1; int dy = y2 - y1;	int dz = z2 - z1;
	//int adx = abs(dx); int ady = abs(dy); int adz = abs(dz);
	int signx = dx > 0 ? 1 : -1; 
	int signy = dy > 0 ? 1 : -1; 
	int signz = dz > 0 ? 1 : -1;
	// absolute distance/abs
	int adx = dx*signx; int ady = dy*signy; int adz = dz*signz;

	//std::cout << "adx " << adx << " ady " << ady << " adz " << adz << "\n";

	int axis = 0;
	if (ady >= adx && ady >= adz) axis = 1;
	else if (adz >= adx && adz >= ady) axis = 2;
	
	if (axis == 0)	// x axis
	{
		for (int x = 0; x <= adx; x++)
		{
			int y, z;

			if (adx == 0)
			{
				y = z = 0;
			}
			else
			{
				y = x*dy / adx;
				z = x*dz / adx;
			}
			
			int cell_idx = (x1+x*signx) + (y1+y)*grid_width_ + (z1+z)*grid_width_*grid_depth_;

			if (cell_idx < grid_.size())
			{
				if ( grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
				{
					if (set_occupy)
					{
						grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
						addEdgeDescriptor2Cell(edge_vec, cell_idx);
						num_voxelized++;
					}

					if (save_points)
					{
						RefPoint rp;
						rp.coordinates[0] = x1 + x*signx;
						rp.coordinates[1] = y1 + y;
						rp.coordinates[2] = z1 + z;
						saved_points_grid_frame.push_back(rp);
					}
				}
				
				if (set_swept_volume && grid_[cell_idx].sweptVolumneCounter != prmce_swept_volume_counter_)
				{
					grid_[cell_idx].sweptVolumneCounter = prmce_swept_volume_counter_;
					num_voxelized++;
					if (grid_[cell_idx].isOccupiedCounter == prmce_round_counter_) 
						prmce_collision_found_ = true;
				}
			}
			else std::cout << "x axis p1" << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2]
				<< " p2 " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";
		}
	}
	else if (axis == 1)	// y axis
	{
		for (int y = 0; y <= ady; y++)
		{
			int x, z;

			if (ady == 0)
			{
				x = z = 0;
			}
			else
			{
				x = y*dx / ady;
				z = y*dz / ady;
			}

			int cell_idx = (x1 + x) + (y1 + y*signy)*grid_width_ + (z1 + z)*grid_width_*grid_depth_;

			if (cell_idx < grid_.size())
			{
				if (grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
				{
					if (set_occupy)
					{
						grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
						addEdgeDescriptor2Cell(edge_vec, cell_idx);
						num_voxelized++;
					}

					if (save_points)
					{
						RefPoint rp;
						rp.coordinates[0] = x1 + x;
						rp.coordinates[1] = y1 + y*signy;
						rp.coordinates[2] = z1 + z;
						saved_points_grid_frame.push_back(rp);
					}
				}

				if (set_swept_volume && grid_[cell_idx].sweptVolumneCounter != prmce_swept_volume_counter_)
				{
					grid_[cell_idx].sweptVolumneCounter = prmce_swept_volume_counter_;
					num_voxelized++;
					if (grid_[cell_idx].isOccupiedCounter == prmce_round_counter_) 
						prmce_collision_found_ = true;
				}
			}
			else std::cout << "y axis p1" << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2]
				<< " p2 " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";
		}

		//std::cout << "\n";
	}
	else	// z axis
	{
		for (int z = 0; z <= adz; z++)
		{
			int x, y;

			if (adz == 0)
			{
				x = y = 0;
			}
			else
			{
				x = z*dx / adz;
				y = z*dy / adz;
			}
			
			int cell_idx = (x1 + x) + (y1 + y)*grid_width_ + (z1 + z*signz)*grid_width_*grid_depth_;

			if (cell_idx < grid_.size())
			{
				if (grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
				{
					if (set_occupy)
					{
						grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
						addEdgeDescriptor2Cell(edge_vec, cell_idx);
						num_voxelized++;
					}

					if (save_points)
					{
						RefPoint rp;
						rp.coordinates[0] = x1 + x;
						rp.coordinates[1] = y1 + y;
						rp.coordinates[2] = z1 + z*signz;
						saved_points_grid_frame.push_back(rp);
					}
				}

				if (set_swept_volume && grid_[cell_idx].sweptVolumneCounter != prmce_swept_volume_counter_)
				{
					grid_[cell_idx].sweptVolumneCounter = prmce_swept_volume_counter_;
					num_voxelized++;
					if (grid_[cell_idx].isOccupiedCounter == prmce_round_counter_) 
						prmce_collision_found_ = true;
				}
			}
			else std::cout << "z axis p1" << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2]
				<< " p2 " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";
		}
	}
	return num_voxelized;
}

int PathPlanner::voxelizeOBB(OBB & obb, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume=false)
{
	// use the rectangle (p1, p2, p3, p4) parallel to yz plane
	// p1-p2 needs to be parallel to p3-p4
	RefPoint p1, p2, p3, p4;
	int num_voxelized = 0;

	Eigen::Vector3f diag2_obb(obb.a);

	// p1 
	Eigen::Vector3f diag2_base = obb.A*diag2_obb;
	for (int i = 0; i < 3; i++) p1.coordinates[i] = obb.C(i) - diag2_base(i);

	// p2
	diag2_obb(1) *= -1.f;
	diag2_base = obb.A*diag2_obb;
	for (int i = 0; i < 3; i++) p2.coordinates[i] = obb.C(i) - diag2_base(i);

	// p3
	diag2_obb(1) *= -1.f; diag2_obb(2) *= -1.f;
	diag2_base = obb.A*diag2_obb;
	for (int i = 0; i < 3; i++) p3.coordinates[i] = obb.C(i) - diag2_base(i);

	// p4
	diag2_obb(1) *= -1.f;
	diag2_base = obb.A*diag2_obb;
	for (int i = 0; i < 3; i++) p4.coordinates[i] = obb.C(i) - diag2_base(i);

	std::vector<RefPoint> p1p2_grid_frame;
	std::vector<RefPoint> p3p4_grid_frame;

	std::vector<RefPoint> rectangle_grid_frame;

	voxelizeLine(p1, p2, p1p2_grid_frame, edge_vec, true, true, false, false);
	voxelizeLine(p3, p4, p3p4_grid_frame, edge_vec, true, true, false, false);

	/*std::cout << "p1p2 size " << p1p2_grid_frame.size() << "\n";
	std::cout << "p3p4 size " << p3p4_grid_frame.size() << "\n";

	if (p1p2_grid_frame.size() != p3p4_grid_frame.size())
		std::cout << "p1p2 != p3p4 \n";
		*/
	//return;

	int line_size = p1p2_grid_frame.size() < p3p4_grid_frame.size() ? p1p2_grid_frame.size() : p3p4_grid_frame.size();

	// voxelize rectangle face
	for (int i = 0; i < line_size; i++)	voxelizeLine(p1p2_grid_frame[i], p3p4_grid_frame[i], rectangle_grid_frame, edge_vec, false, true, false, false);

	//return;
	int count = 0;
	bool set_occupy = !set_swept_volume;
	// extrusion along x positive 
	for (int i = 0; i < rectangle_grid_frame.size(); i++)
	{
		// point on the rectangle, transform back to robot base frame
		RefPoint rp1;
		rp1.coordinates[0] = ((rectangle_grid_frame[i].coordinates[0]+0.5f)*cell_size_ + (float)grid_offset_x_)*0.01f;
		rp1.coordinates[1] = ((rectangle_grid_frame[i].coordinates[1]+0.5f)*cell_size_ + (float)grid_offset_y_)*0.01f;
		rp1.coordinates[2] = ((rectangle_grid_frame[i].coordinates[2]+0.5f)*cell_size_ + (float)grid_offset_z_)*0.01f;

		// end point in robot base frame
		RefPoint rp2;
		for (int j = 0; j < 3; j++)	rp2.coordinates[j] = rp1.coordinates[j] + 2.0f*obb.a(0)*obb.A.col(0)(j);

		num_voxelized += voxelizeLine(rp1, rp2, rectangle_grid_frame, edge_vec, true, false, set_occupy, set_swept_volume);
	}

	return num_voxelized;
}

int PathPlanner::voxelizeArmConfig(std::vector<OBB> & arm_obbs, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume=false)
{
	int num_voxelized = 0;
	for (int i = start_check_obb_idx_; i < arm_obbs.size(); i++) num_voxelized += voxelizeOBB(arm_obbs[i], edge_vec, set_swept_volume);
	return num_voxelized; 
}

void PathPlanner::sweptVolume(float* joint_pos1, float* joint_pos2, std::vector<prmceedge_descriptor> & edge_vec)
{
	float inter_joint_pos[6];

	// bisection style
	for (float layer_alpha = 0.5f; /*layer_alpha > 0.001f*/; layer_alpha *= 0.5f)
	{
		int num_voxelized = 0;

		for (float alpha = layer_alpha; alpha < 1.0f; alpha += 2.f*layer_alpha)
		{
			for (int i = 0; i < 6; i++)	inter_joint_pos[i] = (alpha*joint_pos1[i] + (1.0f - alpha)*joint_pos2[i]);

			std::vector<RefPoint> reference_points;
			std::vector<Eigen::Matrix3f> rot_mats;
			std::vector<OBB> arm_obbs;

			computeReferencePointsOnArm(inter_joint_pos, reference_points, rot_mats);
			
			getArmOBBModel(reference_points, rot_mats, arm_obbs);

			num_voxelized += voxelizeArmConfig(arm_obbs, edge_vec);
		}

		//std::cout << "layer " << layer_alpha << " num vox: " << num_voxelized << "\n";

		if (num_voxelized == 0)	break;
	}
}

bool PathPlanner::selfCollisionBetweenTwoConfigs(float* config1, float* config2)
{
	float inter_joint_pos[6];

	// normalize
	float alpha_step = 0.01f;

	// bisection style
	for (float layer_alpha = 0.5f; layer_alpha >= alpha_step; layer_alpha *= 0.5f)
	{
		for (float alpha = layer_alpha; alpha < 1.0f; alpha += 2.f*layer_alpha)
		{
			for (int i = 0; i < 6; i++)	inter_joint_pos[i] = (alpha*config1[i] + (1.0f - alpha)*config2[i]);

			if (selfCollision(inter_joint_pos)) return true;
		}
	}

	return false;
}

void PathPlanner::PRMCEPreprocessing()
{
	// INITIALIZATION
	clock_t tic = clock();
	random_nodes_buffer_ = new float[num_nodes_ * 6];

	// num_nodes*six joints*three coordinates(x,y,z)
	reference_points_buffer_ = new float[num_nodes_*num_ref_points_ * 3];

	start_end_ref_points_ = new float[2 * 3 * num_ref_points_];

	initGrid(400, 400, 300, 4, -200, -200, -100);
	clock_t toc = clock();
	printf("Init Grid Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	// GENERATE RANDOM SELF-COLLISION-FREE CONFIGURATIONS
	distri_vec_.resize(6);
	// unit: rad
	std::uniform_real_distribution<float> d1((float)joint_range_[0], (float)joint_range_[1]);	// Base
	std::uniform_real_distribution<float> d2((float)joint_range_[2], (float)joint_range_[3]);	// Shoulder
	std::uniform_real_distribution<float> d3((float)joint_range_[4], (float)joint_range_[5]);	// Elbow
	std::uniform_real_distribution<float> d4((float)joint_range_[6], (float)joint_range_[7]);	// Wrist 1
	std::uniform_real_distribution<float> d5((float)joint_range_[8], (float)joint_range_[9]);	// Wrist 2
	std::uniform_real_distribution<float> d6((float)joint_range_[10], (float)joint_range_[11]);		// Wrist3

	distri_vec_[0] = d1;
	distri_vec_[1] = d2;
	distri_vec_[2] = d3;
	distri_vec_[3] = d4;
	distri_vec_[4] = d5;
	distri_vec_[5] = d6;

	float joint_array6[6];
	int fail_count = 0;

	for (int i = 0; i < num_nodes_;)
	{
		// random set of joint pos
		for (int j = 0; j < 6; j++) joint_array6[j] = distri_vec_[j](rand_gen_);

		if (!selfCollision(joint_array6))
		{
			memcpy(random_nodes_buffer_ + i * 6, joint_array6, 6 * sizeof(float));
			//if (i % 50 == 0) std::cout << "random node " << i << "\n";
			i++;
		}
		else fail_count++;
	}

	toc = clock();
	printf("random configs Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	std::cout << "fail count " << fail_count << "\n";

	tic = clock();
	// FORWARD KINEMATICS FOR ALL RANDOM CONFIGURATIONS AND COMPUTE REFERENCE POINTS ON THE ARM
	float* tmp_random_nodes_buffer_ptr = random_nodes_buffer_;
	std::vector<std::vector<RefPoint>> ref_points_vec;
	std::vector<std::vector<Eigen::Matrix3f>> rot_mats_vec;

	for (int i = 0; i < num_nodes_; i++)
	{
		std::vector<RefPoint> ref_points;
		std::vector<Eigen::Matrix3f> rot_mats;
		
		computeReferencePointsOnArm(tmp_random_nodes_buffer_ptr, ref_points, rot_mats);

		ref_points_vec.push_back(ref_points);
		rot_mats_vec.push_back(rot_mats);

		// skip the first 2 ref points, they dont move
		for (int j = 0; j < num_ref_points_; j++)
		{
			float* ptr = reference_points_buffer_ + i * num_ref_points_ * 3 + j * 3;
			*ptr = ref_points[j+2].coordinates[0];
			*(++ptr) = ref_points[j+2].coordinates[1];
			*(++ptr) = ref_points[j+2].coordinates[2];
		}

		tmp_random_nodes_buffer_ptr += 6;
	}

	toc = clock();
	printf("referece points Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	// COMPUTE K NEAREST NEIGHBORS
	flann::Matrix<float> reference_points_mat = flann::Matrix<float>(reference_points_buffer_, num_nodes_, 3 * num_ref_points_);

	// L2 norm of reference points on arm in workspace
	referen_point_index_ = new flann::Index<flann::L2<float>>(reference_points_mat, flann::KDTreeIndexParams(4));

	referen_point_index_->buildIndex();

	flann::Matrix<float> ref_p_query_mat = flann::Matrix<float>(reference_points_buffer_, num_nodes_, 3 * num_ref_points_);

	// neighbor index
	flann::Matrix<int> ref_p_indices_mat(new int[num_nodes_*ref_p_nn_], num_nodes_, ref_p_nn_);
	// distance 
	flann::Matrix<float> ref_p_dists_mat(new float[num_nodes_*ref_p_nn_], num_nodes_, ref_p_nn_);

	referen_point_index_->knnSearch(ref_p_query_mat, ref_p_indices_mat, ref_p_dists_mat, ref_p_nn_, flann::SearchParams(128));

	toc = clock();
	printf("flann knn Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	// BUILD GRAPH
	// add edge based on radius search
	for (int node_id = 0; node_id < num_nodes_; node_id++)
	{
		//if (node_id % 50 == 0) std::cout << "add edge for node " << node_id << "\n";

		// add an edge between center node and neighbor node
		for (int n_idx = 1; n_idx < ref_p_nn_; n_idx++)
		{
			int neighbor_id = *(ref_p_indices_mat.ptr() + ref_p_nn_*node_id + n_idx);

			// actually squared cuz flann didn't sqrt
			float l2_workspace_dist = *(ref_p_dists_mat.ptr() + ref_p_nn_*node_id + n_idx);

			// test if an edge can be added
			float* node_joint_pos = random_nodes_buffer_ + node_id * num_joints_;
			float* neighbor_joint_pos = random_nodes_buffer_ + neighbor_id * num_joints_;

			// check edge existance first then self collision check
			if (!boost::edge(node_id, neighbor_id, prmcegraph_).second && !selfCollisionBetweenTwoConfigs(node_joint_pos, neighbor_joint_pos))
			{
				// add edge
				prmceedge_descriptor e; bool inserted;
				prmceedge edge(node_id, neighbor_id);

				boost::tie(e, inserted) = boost::add_edge(edge.first, edge.second, prmcegraph_);
				prmcegraph_[e].weight = l2_workspace_dist;	
				prmcegraph_[e].weight_copy = l2_workspace_dist;
			}
		}
	}

	// check number of edges
	//std::pair<prmcegraph_t::edge_iterator, prmcegraph_t::edge_iterator> es = boost::edges(prmcegraph_);
	//std::copy(es.first, es.second, std::ostream_iterator<prmceedge_descriptor>{std::cout, " "}); std::cout << "\n";

	// check connected components
	//connected_component_.resize(num_nodes_);
	//int num_cc = boost::connected_components(prmcegraph_, &connected_component_[0]);

	//for (auto label : connected_component_) std::cout << label << " "; std::cout << "\n";
	//std::cout << "cc num: " << num_cc << "\n";

	toc = clock();
	printf("add edges Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
	std::cout << "num of edges: " << boost::num_edges(prmcegraph_) << "\n";

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
	int counter = 0;
	tic = clock();
	// WORKSPACE MAPPING
	// map each vertex
	for (int node_id = 0; node_id < num_nodes_; node_id++)
	{
		float* config = random_nodes_buffer_ + node_id * num_joints_;

		// add all edges connected to this vertice to the cell 
		prmcegraph_t::out_edge_iterator estart, eend;
		std::tie(estart, eend) = boost::out_edges(node_id, prmcegraph_);
		std::vector<prmceedge_descriptor> edge_vec;

		// get all out edge descriptors
		for (prmcegraph_t::out_edge_iterator eit = estart; eit != eend; eit++) edge_vec.push_back(*eit);

		std::vector<OBB> arm_obbs;

		getArmOBBModel(ref_points_vec[node_id], rot_mats_vec[node_id], arm_obbs);

		voxelizeArmConfig(arm_obbs, edge_vec, false);

		//viewOccupancyGrid(viewer);

		prmce_round_counter_++;
	}
	toc = clock();
	printf("map vertices Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	counter = 0;
	tic = clock();
	// map each edge
	std::pair<prmcegraph_t::edge_iterator, prmcegraph_t::edge_iterator> es = boost::edges(prmcegraph_);
	for (prmcegraph_t::edge_iterator eit = es.first; eit != es.second; eit++)
	{
		float* joint_pos1 = random_nodes_buffer_ + (*eit).m_source*num_joints_;
		float* joint_pos2 = random_nodes_buffer_ + (*eit).m_target*num_joints_;

		std::vector<prmceedge_descriptor> edge_vec; edge_vec.push_back(*eit);
		sweptVolume(joint_pos1, joint_pos2, edge_vec);
		
		//viewOccupancyGrid(viewer);

		// round counter, reset occupancy grid status so that new edges can be added to cell
		prmce_round_counter_++;

		//std::cout << ++counter << "\n";
	}

	toc = clock();
	printf("map edges Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
	path_planner_ready_ = true;
}

void PathPlanner::addPointCloudToOccupancyGrid(PointCloudT::Ptr cloud)
{
	if (!path_planner_ready_)
	{
		std::cout << "Path Planner not loaded!\n";
		return;
	}
	clock_t tic = clock();
	for (int i = 0; i < cloud->points.size(); i++)	blockCells(cloud->points[i]);
	clock_t toc = clock();
	printf("add point cloud Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
}

void PathPlanner::viewOccupancyGrid(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	PointCloudT::Ptr cloud(new PointCloudT);
	int count = 0;
	for (int z = 0; z < grid_height_; z++)
	{
		for (int y = 0; y < grid_depth_; y++)
		{
			for (int x = 0; x < grid_width_; x++)
			{
				int cell_idx = x + y*grid_width_ + z*grid_depth_*grid_width_;

				PathPlanner::Cell cell = grid_[cell_idx];

				if (cell.isOccupiedCounter == prmce_round_counter_)
				{
					PointT p;
					p.x = ((x+0.5f)*cell_size_ + grid_offset_x_)*0.01f; p.y = ((y+0.5f)*cell_size_ + grid_offset_y_)*0.01f; p.z = ((z+0.5f)*cell_size_ + grid_offset_z_)*0.01f;
					p.r = p.g = p.b = 200;
					cloud->points.push_back(p);
					count++;
				}
			}
		}
	}

	std::cout << "vox cloud size: " << count/*cloud->points.size()*/ << "\n";

	viewer->removeAllPointClouds();
	viewer->addPointCloud(cloud);
	viewer->addCoordinateSystem(0.3, "world", 0);
	viewer->spin();
}

bool PathPlanner::planPath(float* start_joint_pos, float* end_joint_pos, bool smooth=false, bool try_direct_path = true)
{
	if (!path_planner_ready_)
	{
		std::cout << "NO Path Planner Loaded!\n";
		return false;
	}

	clock_t tic = clock();
	if (selfCollision(start_joint_pos))
	{
		std::cout << "start config self collision \n";
		return false;
	}

	if (selfCollision(end_joint_pos))
	{
		std::cout << "end config self collision \n";
		return false;
	}

	// check path from start to goal directly
	if (try_direct_path && !collisionCheckTwoConfigs(start_joint_pos, end_joint_pos))
	{
		shortest_path_index_vec_.clear();
		prmce_swept_volume_counter_++;
		return true;
	}

	prmce_swept_volume_counter_++;

	std::vector<RefPoint> reference_points_start, reference_points_end;

	std::vector<Eigen::Matrix3f> rot_mats_start, rot_mats_end;

	computeReferencePointsOnArm(start_joint_pos, reference_points_start, rot_mats_start);

	computeReferencePointsOnArm(end_joint_pos, reference_points_end, rot_mats_end);

	for (int i = 0; i < num_ref_points_; i++) memcpy(start_end_ref_points_ + 3*i, reference_points_start[i].coordinates, 3 * sizeof(float));

	for (int i = 0; i < num_ref_points_; i++) memcpy(start_end_ref_points_ + 3*(num_ref_points_ + i), reference_points_end[i].coordinates, 3 * sizeof(float));

	flann::Matrix<float> query_mat = flann::Matrix<float>(start_end_ref_points_, 2, 3*num_ref_points_);

	// search the nearest neighbor
	flann::Matrix<int> indices_mat(new int[2 * ref_p_nn_], 2, ref_p_nn_);
	flann::Matrix<float> dists_mat(new float[2 * ref_p_nn_], 2, ref_p_nn_);

	// index_ for L2 in Cspace !
	referen_point_index_->knnSearch(query_mat, indices_mat, dists_mat, ref_p_nn_, flann::SearchParams(128));

	//for (int i = 0; i < 2 * ref_p_nn_; i++) std::cout << "index: " << *(indices_mat.ptr() + i) << " distance " << *(dists_mat.ptr() + i) << "\n";// << " cc label " << connected_component_[*(indices_mat.ptr() + i)] << "\n";

	int start = -1;
	int goal = -1;

	// check collision on path
	for (int i = 0; i < ref_p_nn_; i++)
	{
		int neighbor = *(indices_mat.ptr() + i);

		// collision check on edge
		if (!collisionCheckTwoConfigs(start_joint_pos, random_nodes_buffer_ + neighbor*num_joints_))
		{
			start = neighbor;
			break;
		}
		//else std::cout << "collision start with neighbor " << neighbor << "\n";
	}

	prmce_swept_volume_counter_++;

	if (start == -1)
	{
		std::cout << "connection between start and roadmap fail \n";
		return false;
	}

	for (int i = 0; i < ref_p_nn_; i++)
	{
		int neighbor = *(indices_mat.ptr() + ref_p_nn_ + i);

		if (!collisionCheckTwoConfigs(end_joint_pos, random_nodes_buffer_ + neighbor*num_joints_))
		{
			goal = neighbor;
			break;
		}
		//else std::cout << "collision goal with neighbor " << neighbor << "\n";
	}

	prmce_swept_volume_counter_++;

	if (goal == -1)
	{
		std::cout << "connection between goal and roadmap fail \n";
		return false;
	}

	clock_t toc = clock();
	printf("connect start and goal to roadmap Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	std::cout << "start index " << start << " -- goal index " << goal << "\n";

	std::cout << "start config: ";
	for (int i = 0; i < num_joints_; i++)
		std::cout << (*(random_nodes_buffer_ + start*num_joints_ + i))*180./M_PI << " ";
	std::cout << "\n";

	std::cout << "goal config: ";
	for (int i = 0; i < num_joints_; i++)
		std::cout << (*(random_nodes_buffer_ + goal*num_joints_ + i))*180./M_PI << " ";
	std::cout << "\n";

	tic = clock();

	std::vector<prmcevertex_descriptor> p(boost::num_vertices(prmcegraph_));

	std::vector<float> d(boost::num_vertices(prmcegraph_));

	

	/*boost::dijkstra_shortest_paths(prmgraph_, start,
	boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, prmgraph_))).
	distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, prmgraph_))));

	std::cout << "distances and parents:" << std::endl;
	boost::graph_traits < prmgraph_t >::vertex_iterator vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(prmgraph_); vi != vend; ++vi) {
	std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
	std::cout << "parent(" << *vi << ") = " << p[*vi] << std::
	endl;
	}
	std::cout << std::endl;

	std::cout << "Shortest path from " << start << "-" << component[start] << " to " << goal << "-" << component[goal] << ": ";
	*/

	// dijkstra shortest path
	/*bool found_path = false;
	try
	{
		boost::dijkstra_shortest_paths(prmcegraph_, start,
			boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, prmcegraph_))).
				weight_map(boost::get(&PRMCEEdge::weight, prmcegraph_)).
					distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, prmcegraph_))).
						visitor(astar_goal_visitor<prmcevertex_descriptor>(goal)));
	}
	catch (found_goal fg)
	{
		shortest_path_index_vec_.clear();
		for (prmcevertex_descriptor v = goal;; v = p[v])
		{
			shortest_path_index_vec_.push_back(v);
			if (p[v] == v) break;
		}

		std::reverse(std::begin(shortest_path_index_vec_), std::end(shortest_path_index_vec_));

		std::cout << "Dijkstra Shortest path from " << start << " to " << goal <<": \n";
		std::vector<prmcevertex_descriptor>::iterator spi = shortest_path_index_vec_.begin();
		
		for (++spi; spi != shortest_path_index_vec_.end(); ++spi) std::cout << " -> " << *spi;

		std::cout << std::endl << "Total travel distance: " << d[goal] << std::endl;

		found_path = d[goal] < 500.f ? true : false;
	}*/

	// A star shortest path
	bool found_path = false;
	try {
		boost::astar_search_tree
			(prmcegraph_, start,
				distance_heuristic<prmcegraph_t, float, float*, int>(reference_points_buffer_, goal, num_ref_points_*3),
					boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, prmcegraph_))).
						weight_map(boost::get(&PRMCEEdge::weight, prmcegraph_)).
							distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, prmcegraph_))).
								visitor(astar_goal_visitor<prmcevertex_descriptor>(goal)));
	}
	catch (found_goal fg)
	{
		shortest_path_index_vec_.clear();

		for (prmcevertex_descriptor v = goal;; v = p[v])
		{
			shortest_path_index_vec_.push_back(v);
			if (p[v] == v) break;
		}

		std::reverse(std::begin(shortest_path_index_vec_), std::end(shortest_path_index_vec_));

		std::cout << "A* shortest path from " << start << " to " << goal  << ": ";
		std::vector<prmcevertex_descriptor>::iterator spi = shortest_path_index_vec_.begin();
		
		for (++spi; spi != shortest_path_index_vec_.end(); ++spi) std::cout << " -> " << *spi;

		std::cout << std::endl << "Total travel distance: " << d[goal] << std::endl;

		found_path = d[goal] < 500.f ? true : false;
	}

	toc = clock();
	printf("grahp search Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	if (smooth)
	{
		smoothPath();
	}

	delete[] indices_mat.ptr();
	delete[] dists_mat.ptr();

	return found_path;
}

/*
	sweep the arm in occupancy grid
*/
bool PathPlanner::collisionCheckTwoConfigs(float* config1, float* config2)
{
	float inter_joint_pos[6];

	prmce_collision_found_ = false;

	// bisection style
	for (float layer_alpha = 0.5f; /*layer_alpha > 0.001f*/; layer_alpha *= 0.5f)
	{
		int num_voxelized = 0;

		for (float alpha = layer_alpha; alpha < 1.0f; alpha += 2.f*layer_alpha)
		{
			for (int i = 0; i < 6; i++)	inter_joint_pos[i] = (alpha*config1[i] + (1.0f - alpha)*config2[i]);

			std::vector<RefPoint> reference_points;
			std::vector<Eigen::Matrix3f> rot_mats;
			std::vector<OBB> arm_obbs;
			std::vector<prmceedge_descriptor> edge_vec;

			computeReferencePointsOnArm(inter_joint_pos, reference_points, rot_mats);

			getArmOBBModel(reference_points, rot_mats, arm_obbs);

			num_voxelized += voxelizeArmConfig(arm_obbs, edge_vec, true);

			if (prmce_collision_found_) return true;
		}

		//std::cout << "layer " << layer_alpha << " num vox: " << num_voxelized << "\n";

		if (num_voxelized == 0)	break;
	}

	return false;
}

void PathPlanner::savePathPlanner(std::string filename)
{
	clock_t tic = clock();
	//std::ofstream ofs(filename+".dat"); boost::archive::text_oarchive oa(ofs);	// slow, portable 
	std::ofstream ofs(filename + ".dat", std::ios::binary); boost::archive::binary_oarchive oa(ofs);	//fast, non-portable
	oa << *this;

	referen_point_index_->save(filename+".idx");
	
	clock_t toc = clock();
	printf("save path planner Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
}

bool PathPlanner::loadPathPlanner(std::string filename)
{
	if (path_planner_ready_)
	{
		std::cout << "Path Planner Already Load!\n";
		return false;
	}

	clock_t tic = clock();

	bool success = false;

	if (boost::filesystem::exists(filename+".dat") && boost::filesystem::exists(filename + ".idx"))
	{
		prmcegraph_.clear();
		//std::ifstream ifs(filename+".dat"); boost::archive::text_iarchive ia(ifs);
		std::ifstream ifs(filename + ".dat", std::ios::binary); boost::archive::binary_iarchive ia(ifs);
		ia >> *this;

		start_end_ref_points_ = new float[2 * 3 * num_ref_points_];

		// COMPUTE K NEAREST NEIGHBORS
		flann::Matrix<float> reference_points_mat = flann::Matrix<float>(reference_points_buffer_, num_nodes_, 3 * num_ref_points_);

		// L2 norm of reference points on arm in workspace
		referen_point_index_ = new flann::Index<flann::L2<float>>(reference_points_mat, flann::SavedIndexParams(filename+".idx"));

		referen_point_index_->buildIndex();

		success = path_planner_ready_ =true;
	}
	else std::cout << filename << " does not exist!\n";

	clock_t toc = clock();
	printf("load path planner Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	//std::cout << boost::num_vertices(prmcegraph_)<<" edges: "<< boost::num_edges(prmcegraph_) << "\n";

	return success;
}

void PathPlanner::resetOccupancyGrid()
{
	clock_t tic = clock();
	for (int i = 0; i < num_cells_; i++)
	{
		grid_[i].isOccupiedCounter = 0;
		grid_[i].sweptVolumneCounter = 0;
	}

	prmce_round_counter_ = prmce_swept_volume_counter_ = 1;

	clock_t toc = clock();
	printf("reset grid Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
}

void PathPlanner::smoothPath()
{
	if (shortest_path_index_vec_.size() < 3) return;

	clock_t tic = clock();
	int cur_vertex = 0;
	int new_vertex = 2;
	
	std::vector<prmcevertex_descriptor> smooth_path;
	smooth_path.push_back(shortest_path_index_vec_[0]);

	while (new_vertex < shortest_path_index_vec_.size())
	{
		while (new_vertex < shortest_path_index_vec_.size() &&
			!collisionCheckTwoConfigs(random_nodes_buffer_ + shortest_path_index_vec_[cur_vertex]*num_joints_, 
										random_nodes_buffer_ + shortest_path_index_vec_[new_vertex]*num_joints_))
		{
			new_vertex++;
			prmce_swept_volume_counter_++;
		}

		prmce_swept_volume_counter_++;
		cur_vertex = new_vertex - 1;

		smooth_path.push_back(shortest_path_index_vec_[cur_vertex]);

		new_vertex++;
	}
	
	clock_t toc = clock();
	printf("smooth path Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	std::cout << "smooth path: ";
	for (auto e : smooth_path) std::cout << e << "->";
	std::cout << "\n";

	shortest_path_index_vec_.swap(smooth_path);
}

bool PathPlanner::collisionCheckForSingleConfig(float* config)
{
	if (selfCollision(config))
	{
		std::cout << "self collision\n";
		return true;
	}

	// collision check with environment
	prmce_collision_found_ = false;

	std::vector<RefPoint> reference_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<OBB> arm_obbs;
	std::vector<prmceedge_descriptor> edge_vec;

	computeReferencePointsOnArm(config, reference_points, rot_mats);

	getArmOBBModel(reference_points, rot_mats, arm_obbs);

	int num_voxelized = voxelizeArmConfig(arm_obbs, edge_vec, true);

	prmce_swept_volume_counter_++;

	return prmce_collision_found_;
}