#include "PathPlanner.h"

PathPlanner::PathPlanner() :
	num_nodes_(800),	//must be even 800/10
	ref_p_nn_(10),
	prmcegraph_(num_nodes_),
	num_joints_(6),
	//rand_gen_(time(0))
	//rand_gen_(1),
	num_ref_points_(9),
	prmce_round_counter_(1),
	prmce_swept_volume_counter_(1),
	prmce_collision_found_(false),
	path_planner_ready_(false)
{
	// DH parameters
	a_[0] = a_[3] = a_[4] = a_[5] = 0.f; a_[1] = -0.612f; a_[2] = -0.5732f;
	d_[1] = d_[2] = 0.; d_[0] = 0.1273; d_[3] = 0.163941; d_[4] = 0.1157; d_[5] = 0.0922; //UR
	alpha_[1] = alpha_[2] = alpha_[5] = 0.f; alpha_[0] = alpha_[3] = 1.570796327f; alpha_[4] = -1.570796327f;
}

PathPlanner::~PathPlanner()
{
	if(random_nodes_buffer_ != NULL) delete[] random_nodes_buffer_;
	if(reference_points_buffer_ != NULL) delete[] reference_points_buffer_;
	if(start_end_ref_points_ != NULL) delete[] start_end_ref_points_;
}

/*
	http://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/actual-center-of-mass-for-robot-17264/
	https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
*/
Eigen::Matrix4f PathPlanner::constructDHMatrix(int target_joint_id, float target_joint_pos)
{
	Eigen::Matrix4d DH_mat;
	DH_mat = Eigen::Matrix4d::Identity();
	double cos_target = cos(target_joint_pos);
	double sin_target = sin(target_joint_pos);
	double cos_alp_tar = cos(alpha_[target_joint_id]);
	double sin_alp_tar = sin(alpha_[target_joint_id]);

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

	return DH_mat.cast<float>();
}

void PathPlanner::forwardKinematicsUR10(float* joint_array6)
{
	for (int i = 0; i < num_joints_; i++)	DH_mat_vec_[i] = constructDHMatrix(i, joint_array6[i]);
	fk_mat_ = DH_mat_vec_[0];
	for (int i = 1; i < num_joints_; i++) fk_mat_ = fk_mat_*DH_mat_vec_[i];
}

void PathPlanner::forwardKinematicsUR10ROS(float* joint_array6)
{
	const double d1 = 0.1273;
	const double a2 = -0.612;
	const double a3 = -0.5723;
	const double d4 = 0.163941;
	const double d5 = 0.1157;
	const double d6 = 0.0922;

	double transform[16];
	double j[6];

	for (int i = 0; i < 6; i++ )
	{
		j[i] = (double)joint_array6[i];
	}

	double* q = j;
	double *T = transform;

	double s1 = sin(*q), c1 = cos(*q); q++;
	double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
	double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
	q234 += *q; q++;
	double s5 = sin(*q), c5 = cos(*q); q++;
	double s6 = sin(*q), c6 = cos(*q);
	double s234 = sin(q234), c234 = cos(q234);
	*T = ((c1*c234 - s1*s234)*s5) / 2.0 - c5*s1 + ((c1*c234 + s1*s234)*s5) / 2.0; T++;
	*T = (c6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0) -
		(s6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0); T++;
	*T = (-(c6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0 -
		s6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0)); T++;
	*T = ((d5*(s1*c234 - c1*s234)) / 2.0 - (d5*(s1*c234 + c1*s234)) / 2.0 -
		d4*s1 + (d6*(c1*c234 - s1*s234)*s5) / 2.0 + (d6*(c1*c234 + s1*s234)*s5) / 2.0 -
		a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
	*T = c1*c5 + ((s1*c234 + c1*s234)*s5) / 2.0 + ((s1*c234 - c1*s234)*s5) / 2.0; T++;
	*T = (c6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0) +
		s6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0)); T++;
	*T = (c6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0) -
		s6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0)); T++;
	*T = ((d5*(c1*c234 - s1*s234)) / 2.0 - (d5*(c1*c234 + s1*s234)) / 2.0 + d4*c1 +
		(d6*(s1*c234 + c1*s234)*s5) / 2.0 + (d6*(s1*c234 - c1*s234)*s5) / 2.0 + d6*c1*c5 -
		a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
	*T = ((c234*c5 - s234*s5) / 2.0 - (c234*c5 + s234*s5) / 2.0); T++;
	*T = ((s234*c6 - c234*s6) / 2.0 - (s234*c6 + c234*s6) / 2.0 - s234*c5*c6); T++;
	*T = (s234*c5*s6 - (c234*c6 + s234*s6) / 2.0 - (c234*c6 - s234*s6) / 2.0); T++;
	*T = (d1 + (d6*(c234*c5 - s234*s5)) / 2.0 + a3*(s2*c3 + c2*s3) + a2*s2 -
		(d6*(c234*c5 + s234*s5)) / 2.0 - d5*c234); T++;
	*T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;

	for (int y = 0; y < 4; y++)
	{
		for (int x = 0; x < 4; x++)
		{
			std::cout << transform[y * 4 + x]<<" ";
		}
		std::cout << "\n";
	}
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
#if 0
		int x = ((int)(point_in_arm_base.x*100.f) - grid_offset_x_) / cell_size_;
		int y = ((int)(point_in_arm_base.y*100.f) - grid_offset_y_) / cell_size_;
		int z = ((int)(point_in_arm_base.z*100.f) - grid_offset_z_) / cell_size_;
#endif
		int x = ((int)(-point_in_arm_base.x*100.f) + grid_offset_x_) / cell_size_;
		int y = ((int)(-point_in_arm_base.y*100.f) + grid_offset_y_) / cell_size_;
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

	// origin, no need to add rotation matrix
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = 0.f; 
	reference_points.push_back(ref_p);

	// frame 1 on ur10 dh figure
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = accum_dh_mat(i, 3);  
	reference_points.push_back(ref_p);
	rot_mats.push_back(accum_dh_mat.block<3,3>(0,0));

	// frame 1, the other side, z positive
	vec(0) = 0.f; vec(1) = 0.f; vec(2) = 0.2f, vec(3) = 1.f; vec = accum_dh_mat * vec;
	for (int i = 0; i < 3; i++) ref_p.coordinates[i] = vec(i); 
	reference_points.push_back(ref_p);

	// frame 2, the other side, z positive
	dh_mat = constructDHMatrix(1, joint_pos[1]); accum_dh_mat = accum_dh_mat * dh_mat;
	vec(0) = 0.f; vec(1) = 0.f; vec(2) = 0.2f, vec(3) = 1.f; vec = accum_dh_mat * vec;
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

	//long probe
	RefPoint tmp_rp;
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_p.coordinates[i] + rot_mats[5](i, 0)*probe_position[0] + rot_mats[5](i, 1)*probe_position[1] + rot_mats[5](i, 2)*probe_position[2];
	reference_points.push_back(tmp_rp);

	//cameras
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_p.coordinates[i] + rot_mats[5](i, 0)*(0.01524f) + rot_mats[5](i, 1)*(0.082804f) + rot_mats[5](i, 2)*(0.2f);
	reference_points.push_back(tmp_rp);
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

void PathPlanner::constructOBB(RefPoint & p0, RefPoint & p1, Eigen::Matrix3f rot, float radius_1, float radius_2, int axis, OBB & obb)
{
	Eigen::Vector3f point0(p0.coordinates[0], p0.coordinates[1], p0.coordinates[2]);
	Eigen::Vector3f point1(p1.coordinates[0], p1.coordinates[1], p1.coordinates[2]);

	// center
	obb.C = 0.5f*(point0 + point1);
	
	obb.A = rot;

	// extents
	obb.a(axis) = (point0 - point1).norm()*0.5f;
	if (axis == 0)
	{
		obb.a(1) = radius_1;
		obb.a(2) = radius_2;
	}
	else if (axis == 1)
	{
		obb.a(2) = radius_1;
		obb.a(0) = radius_2;
	}
	else if (axis == 2)
	{
		obb.a(0) = radius_1;
		obb.a(1) = radius_2;
	}
}

void PathPlanner::getArmOBBModel(std::vector<RefPoint> ref_points, std::vector<Eigen::Matrix3f> & rot_mats, std::vector<OBB> & arm_obbs)
{
	arm_obbs.clear();

	RefPoint tmp_rp, tmp_rp1;

	// 0. add rover base
	OBB obb;
	obb.C << 0.0635f, 0.f, -0.48f;
	obb.A = Eigen::Matrix3f::Identity();
	obb.a << 0.1397f, 0.254f, 0.48f;
	arm_obbs.push_back(obb);

	// 1. add rover base 1
	obb.C << 0.508, 0.f, -0.5969f;
	obb.A = Eigen::Matrix3f::Identity();
	obb.a << 0.3048f, 0.254f, 0.3683f;
	arm_obbs.push_back(obb);

	// 2. add rover base 2
	tmp_rp.coordinates[0] = 0.2032f; tmp_rp.coordinates[1] = 0.f; tmp_rp.coordinates[2] = 0.f;
	tmp_rp1.coordinates[0] = 0.3556f; tmp_rp1.coordinates[1] = 0.f; tmp_rp1.coordinates[2] = -0.2286f;
	Eigen::Matrix3f rot = Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitY()).matrix();
	constructOBB(tmp_rp, tmp_rp1, rot, 0.254f, 0.01f, 0, obb);
	arm_obbs.push_back(obb);

	// 3. add chamber window top wall, robot arm base center to front edge 16.5cm, robot-to-window 20cm, wall thickness 5cm
	// floor-to-base height 0.964m, 
	obb.C << -0.3f, 0.f,  1.07f + 0.2f;
	obb.A = Eigen::Matrix3f::Identity();
	obb.a << 0.05f, 1.3f, 0.2f;
	arm_obbs.push_back(obb);

	// 4. add chamber window bottom wall
	obb.C << -0.3f, 0.f, -0.34f - 0.3f;
	obb.a << 0.05f, 1.3f,  0.3f;
	arm_obbs.push_back(obb);

	// 5. window top wall in top-view mode y = -0.3, change it to window top wall in side-view mode, use y = 0.76
	obb.C << -0.3f, 0.f, 1.07f + 0.2f;
	obb.a << 0.05f, 1.3f,  0.2f;
	arm_obbs.push_back(obb);

	// 6. add chamber inside wall
	obb.C << -1.13f, 0.f,  0.17f;
	obb.a << 0.04f, 1.3f, 1.4f;
	arm_obbs.push_back(obb);

	// 7. add chamber table
	obb.C << -0.7f, 0.f, -0.732f;
	obb.a << 0.35f, 1.3f, 0.01f;
	arm_obbs.push_back(obb);

	// 8. add frame 1 arm, UR10 DH figure, y axis
	ref_points[1].coordinates[2] = 0.21f;
	constructOBB(ref_points[0], ref_points[1], rot_mats[0], 0.11f, 0.11f, 1, obb);
	arm_obbs.push_back(obb);

	// 9. 1st long arm, frame 2
	for (int i = 0; i < 3; i++) ref_points[2].coordinates[i] += rot_mats[1](i, 0)*0.06f;
	for (int i = 0; i < 3; i++) ref_points[3].coordinates[i] -= rot_mats[1](i, 0)*0.06f;
	constructOBB(ref_points[2], ref_points[3], rot_mats[1], 0.089f, 0.089f, 0, obb);
	arm_obbs.push_back(obb);

	// 10. 2nd long arm, frame 3
	for (int i = 0; i < 3; i++) ref_points[4].coordinates[i] += rot_mats[2](i, 0)*0.06f;
	for (int i = 0; i < 3; i++) ref_points[5].coordinates[i] -= rot_mats[2](i, 0)*0.03f;	//0.03 original
	constructOBB(ref_points[4], ref_points[5], rot_mats[2], 0.055f, 0.055f, 0, obb);
	arm_obbs.push_back(obb);

	// 11. frame 4
	for (int i = 0; i < 3; i++) ref_points[6].coordinates[i] -= rot_mats[3](i, 2)*0.06f;
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[7].coordinates[i] - rot_mats[3](i, 2)*0.051f;
	constructOBB(ref_points[6], tmp_rp, rot_mats[3], 0.047f, 0.047f, 2, obb);
	arm_obbs.push_back(obb);

	// 12. frame 5
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[7].coordinates[i] - rot_mats[4](i, 2)*0.045f;
	constructOBB(tmp_rp, ref_points[8], rot_mats[4], 0.045f, 0.045f, 2, obb);
	arm_obbs.push_back(obb);

	// 13. sensor block
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.10f + rot_mats[5](i, 1)*0.07f + rot_mats[5](i, 2)*0.1f;
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] - rot_mats[5](i, 0)*0.20f + rot_mats[5](i, 1)*0.07f + rot_mats[5](i, 2)*0.1f;
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.11f, 0.1f, 0, obb);
	arm_obbs.push_back(obb);

	// 14. thermal camera (the part above the tool flange)
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] - rot_mats[5](i, 0)*0.0313f + rot_mats[5](i, 2)*0.0f + rot_mats[5](i, 1)*0.1434f;
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] - rot_mats[5](i, 0)*0.0313f - rot_mats[5](i, 2)*0.12f + rot_mats[5](i, 1)*0.1434f;
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.045f, 0.045f, 2, obb);
	arm_obbs.push_back(obb);

	// 15. probe cylinder
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*cylinder_back_position[0] + rot_mats[5](i, 1)*cylinder_back_position[1];
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*cylinder_back_position[0] + rot_mats[5](i, 1)*cylinder_back_position[1] + rot_mats[5](i, 2)*cylinder_back_position[2];
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.03f, 0.04f, 2, obb);
	arm_obbs.push_back(obb);

	//16. line light
	for (int i = 0; i < 3; i++) tmp_rp.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.20f + rot_mats[5](i, 1)*0.07f + rot_mats[5](i, 2)*0.13f;
	for (int i = 0; i < 3; i++) tmp_rp1.coordinates[i] = ref_points[8].coordinates[i] + rot_mats[5](i, 0)*0.10f + rot_mats[5](i, 1)*0.07f + rot_mats[5](i, 2)*0.13f;
	constructOBB(tmp_rp1, tmp_rp, rot_mats[5], 0.11f, 0.07f, 0, obb);
	arm_obbs.push_back(obb);
}

bool PathPlanner::selfCollision(float* joint_pos, bool pre_processing_stage)
{
	std::vector<RefPoint> reference_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<OBB> arm_obbs;

	computeReferencePointsOnArm(joint_pos, reference_points, rot_mats);

	//prevent hand hit the back and side walls
	if (pre_processing_stage)
	{
		if (reference_points.back().coordinates[0] > tcp_x_limit_
			|| std::abs(reference_points.back().coordinates[1]) > 1.0	// camera
			|| std::abs(reference_points[reference_points.size()-2].coordinates[1]) > 1.0 //probe
			)
		{
			//std::cout << "hand in the back too far\n";
			return true;
		}
	}
	//else
	//{
	//	if (reference_points.back().coordinates[1] > 0.9f)
	//	{
	//		//std::cout << "hand in the back too far\n";
	//		return true;
	//	}
	//}

	getArmOBBModel(reference_points, rot_mats, arm_obbs);

	for (int i = start_check_obb_idx_; i < arm_obbs.size(); i++)
	{
		const int end_idx = std::min(i, end_check_obb_idx_);

		for (int j = 0; j < end_idx; j++)
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
#if 0
		// convert to cm and floor, to grid coordinates
		x1 = ((int)(p1.coordinates[0] * 100.f) - grid_offset_x_) / cell_size_;
		y1 = ((int)(p1.coordinates[1] * 100.f) - grid_offset_y_) / cell_size_;
		z1 = ((int)(p1.coordinates[2] * 100.f) - grid_offset_z_) / cell_size_;

		x2 = ((int)(p2.coordinates[0] * 100.f) - grid_offset_x_) / cell_size_;
		y2 = ((int)(p2.coordinates[1] * 100.f) - grid_offset_y_) / cell_size_;
		z2 = ((int)(p2.coordinates[2] * 100.f) - grid_offset_z_) / cell_size_;
#endif
		// convert to cm and floor, to grid coordinates
		x1 = ((int)(-p1.coordinates[0] * 100.f) + grid_offset_x_) / cell_size_;
		y1 = ((int)(-p1.coordinates[1] * 100.f) + grid_offset_y_) / cell_size_;
		z1 = ((int)(p1.coordinates[2] * 100.f) - grid_offset_z_) / cell_size_;

		x2 = ((int)(-p2.coordinates[0] * 100.f) + grid_offset_x_) / cell_size_;
		y2 = ((int)(-p2.coordinates[1] * 100.f) + grid_offset_y_) / cell_size_;
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

	const int dx = x2 - x1; 
	const int dy = y2 - y1;	
	const int dz = z2 - z1;
	//int adx = abs(dx); int ady = abs(dy); int adz = abs(dz);
	const int signx = dx > 0 ? 1 : -1; 
	const int signy = dy > 0 ? 1 : -1; 
	const int signz = dz > 0 ? 1 : -1;
	// absolute distance/abs
	const int adx = dx*signx; 
	const int ady = dy*signy; 
	const int adz = dz*signz;

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
			
			const int new_x = x1 + x*signx;

			const int new_y = y1 + y;

			const int new_z = z1 + z;

			const int cell_idx = new_x + new_y*grid_width_ + new_z*grid_width_*grid_depth_;

			if (!save_points)	// when not saving points, it would do set_occupy or set_swept_volume, needs to check range
			{
				if (new_x >= grid_width_ || new_x < 0) continue;
				if (new_y >= grid_depth_ || new_y < 0) continue;
				if (new_z >= grid_height_ || new_z < 0) continue;
				if (cell_idx >= grid_.size()) continue;

				if (set_occupy && grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
				{
					grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
					addEdgeDescriptor2Cell(edge_vec, cell_idx);
					num_voxelized++;
				}

				if (set_swept_volume && grid_[cell_idx].sweptVolumneCounter != prmce_swept_volume_counter_)
				{
					grid_[cell_idx].sweptVolumneCounter = prmce_swept_volume_counter_;
					num_voxelized++;
					if (grid_[cell_idx].isOccupiedCounter == prmce_round_counter_)
						prmce_collision_found_ = true;
				}
			}
			else
			{
				RefPoint rp;
				rp.coordinates[0] = x1 + x*signx;
				rp.coordinates[1] = y1 + y;
				rp.coordinates[2] = z1 + z;
				saved_points_grid_frame.push_back(rp);
			}
		//	else std::cout << "x axis p1" << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2]
			//	<< " p2 " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";
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

			const int new_x = x1 + x;

			const int new_y = y1 + y*signy;

			const int new_z = z1 + z;

			const int cell_idx = new_x + new_y*grid_width_ + new_z*grid_width_*grid_depth_;

			if (!save_points)
			{

				if (new_x >= grid_width_ || new_x < 0) continue;
				if (new_y >= grid_depth_ || new_y < 0) continue;
				if (new_z >= grid_height_ || new_z < 0) continue;
				if (cell_idx >= grid_.size()) continue;

				if (set_occupy && grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
				{
					grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
					addEdgeDescriptor2Cell(edge_vec, cell_idx);
					num_voxelized++;
				}

				if (set_swept_volume && grid_[cell_idx].sweptVolumneCounter != prmce_swept_volume_counter_)
				{
					grid_[cell_idx].sweptVolumneCounter = prmce_swept_volume_counter_;
					num_voxelized++;
					if (grid_[cell_idx].isOccupiedCounter == prmce_round_counter_)
						prmce_collision_found_ = true;
				}
			}
			else
			{
				RefPoint rp;
				rp.coordinates[0] = x1 + x;
				rp.coordinates[1] = y1 + y*signy;
				rp.coordinates[2] = z1 + z;
				saved_points_grid_frame.push_back(rp);
			}
		//	else std::cout << "y axis p1" << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2]
			//	<< " p2 " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";
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

			const int new_x = x1 + x;
			
			const int new_y = y1 + y;
			
			const int new_z = z1 + z*signz;

			const int cell_idx = new_x + new_y*grid_width_ + new_z*grid_width_*grid_depth_;

			if (!save_points)
			{
				if (new_x >= grid_width_ || new_x < 0) continue;
				if (new_y >= grid_depth_ || new_y < 0) continue;
				if (new_z >= grid_height_ || new_z < 0) continue;
				if (cell_idx >= grid_.size()) continue;

				if (set_occupy && grid_[cell_idx].isOccupiedCounter != prmce_round_counter_)
				{
					grid_[cell_idx].isOccupiedCounter = prmce_round_counter_;
					addEdgeDescriptor2Cell(edge_vec, cell_idx);
					num_voxelized++;
				}

				if (set_swept_volume && grid_[cell_idx].sweptVolumneCounter != prmce_swept_volume_counter_)
				{
					grid_[cell_idx].sweptVolumneCounter = prmce_swept_volume_counter_;
					num_voxelized++;
					if (grid_[cell_idx].isOccupiedCounter == prmce_round_counter_)
						prmce_collision_found_ = true;
				}
			}
			else
			{
				RefPoint rp;
				rp.coordinates[0] = x1 + x;
				rp.coordinates[1] = y1 + y;
				rp.coordinates[2] = z1 + z*signz;
				saved_points_grid_frame.push_back(rp);
			}
		//	else std::cout << "z axis p1" << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2]
			//	<< " p2 " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";
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

	//clock_t tic = clock();
	voxelizeLine(p1, p2, p1p2_grid_frame, edge_vec, true, true, false, false);
	voxelizeLine(p3, p4, p3p4_grid_frame, edge_vec, true, true, false, false);

	/*std::cout << "p1p2 size " << p1p2_grid_frame.size() << "\n";
	std::cout << "p3p4 size " << p3p4_grid_frame.size() << "\n";

	if (p1p2_grid_frame.size() != p3p4_grid_frame.size())
		std::cout << "p1p2 != p3p4 \n";*/
#if 0
	if (p1p2_grid_frame.size() == 0 || p3p4_grid_frame.size() == 0)
	{
		std::cout << "p1p2 size " << p1p2_grid_frame.size() << "\n";
		std::cout << "p3p4 size " << p3p4_grid_frame.size() << "\n";
		std::cout << "p1: " << p1.coordinates[0] << " " << p1.coordinates[1] << " " << p1.coordinates[2] << "\n";
		std::cout << "p2: " << p2.coordinates[0] << " " << p2.coordinates[1] << " " << p2.coordinates[2] << "\n";

		std::cout << "p3: " << p3.coordinates[0] << " " << p3.coordinates[1] << " " << p3.coordinates[2] << "\n";
		std::cout << "p4: " << p4.coordinates[0] << " " << p4.coordinates[1] << " " << p4.coordinates[2] << "\n";
	}
#endif
		
	//return;

	const int line_size = p1p2_grid_frame.size() < p3p4_grid_frame.size() ? p1p2_grid_frame.size() : p3p4_grid_frame.size();

	// voxelize rectangle face
	for (int i = 0; i < line_size; i++)
	{
		//int idx1 = i < p1p2_grid_frame.size() ? i : p1p2_grid_frame.size() - 1;
		//int idx2 = i < p3p4_grid_frame.size() ? i : p3p4_grid_frame.size() - 1;
		voxelizeLine(p1p2_grid_frame[i], p3p4_grid_frame[i], rectangle_grid_frame, edge_vec, false, true, false, false);
	}

	const bool set_occupy = !set_swept_volume;
	// extrusion along x positive 
	for (int i = 0; i < rectangle_grid_frame.size(); i++)
	{
		// point on the rectangle, transform back to robot base frame
		RefPoint rp1;
#if 0
		rp1.coordinates[0] = ((rectangle_grid_frame[i].coordinates[0]+0.5f)*cell_size_ + (float)grid_offset_x_)*0.01f;
		rp1.coordinates[1] = ((rectangle_grid_frame[i].coordinates[1]+0.5f)*cell_size_ + (float)grid_offset_y_)*0.01f;
		rp1.coordinates[2] = ((rectangle_grid_frame[i].coordinates[2]+0.5f)*cell_size_ + (float)grid_offset_z_)*0.01f;
#endif

		rp1.coordinates[0] = ((-rectangle_grid_frame[i].coordinates[0])*cell_size_ + (float)grid_offset_x_)*0.01f;
		rp1.coordinates[1] = ((-rectangle_grid_frame[i].coordinates[1])*cell_size_ + (float)grid_offset_y_)*0.01f;
		rp1.coordinates[2] = ((rectangle_grid_frame[i].coordinates[2])*cell_size_ + (float)grid_offset_z_)*0.01f;

		// end point in robot base frame
		RefPoint rp2;
		for (int j = 0; j < 3; j++)	rp2.coordinates[j] = rp1.coordinates[j] + 2.0f*obb.a(0)*obb.A.col(0)(j);

		num_voxelized += voxelizeLine(rp1, rp2, rectangle_grid_frame, edge_vec, true, false, set_occupy, set_swept_volume);
	}

	return num_voxelized;
}

int PathPlanner::voxelizeArmConfig(std::vector<OBB> & arm_obbs, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume=false, bool shorten_probe=false)
{
	int num_voxelized = 0;

	//for single config collision detection, probe tip touch leaf
	if (shorten_probe)
	{
		arm_obbs[arm_obbs.size() - 1].a(2) -= 0.04f;
	}

	for (int i = start_check_obb_idx_; i < arm_obbs.size(); i++)
		num_voxelized += voxelizeOBB(arm_obbs[i], edge_vec, set_swept_volume);

	return num_voxelized; 
}

void PathPlanner::sweptVolume(float* joint_pos1, float* joint_pos2, std::vector<prmceedge_descriptor> & edge_vec)
{
	float inter_joint_pos[6];

	float max_dist = 0.f;

	for (int i = 0; i < num_joints_; i++)
	{
		float dist = std::abs(joint_pos1[i] - joint_pos2[i]);

		if (dist > max_dist)
			max_dist = dist;
	}

	float min_layer_alpha = 0.01f/180.f*M_PI/max_dist;

	min_layer_alpha = std::min(min_layer_alpha, 0.3f);
	
	// bisection style
	for (float layer_alpha = 0.5f; layer_alpha > min_layer_alpha; layer_alpha *= 0.5f)
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

		//if (num_voxelized == 0)	break;
	}
}

bool PathPlanner::selfCollisionBetweenTwoConfigs(float* config1, float* config2)
{
	//find the max joint displacement
	float max_joint_dist = 0.f;
	int joint_idx = -1;
	for (int i = 0; i < num_joints_; i++)
	{
		float tmp = std::abs(config1[i] - config2[i]);
		if (tmp > max_joint_dist)
		{
			max_joint_dist = tmp;
			joint_idx = i;
		}
	}

	const float min_angle_resolution = M_PI / 90.f;

	float inter_joint_pos[6];

	// normalize
	float alpha_step = min_angle_resolution/ max_joint_dist;
	alpha_step = std::min(alpha_step, 0.3f);

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


/*
https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
Analytical solutions + picking the feasible one
*/
int PathPlanner::inverseKinematics(Eigen::Matrix4d & T, std::vector<int> & ik_sols_vec)
{
	ik_sols_vec.clear();
	const double q6_des = -PI;
	int num_sols = 0;
	/*double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
	double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
	double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;*/
	double T00 = T(0, 0); double T01 = T(0, 1); double T02 = T(0, 2); double T03 = T(0, 3);
	double T10 = T(1, 0); double T11 = T(1, 1); double T12 = T(1, 2); double T13 = T(1, 3);
	double T20 = T(2, 0); double T21 = T(2, 1); double T22 = T(2, 2); double T23 = T(2, 3);

	////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
	double q1[2];
	{
		double A = d6*T12 - T13;
		double B = d6*T02 - T03;
		double R = A*A + B*B;
		if (fabs(A) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
				div = -SIGN(d4)*SIGN(B);
			else
				div = -d4 / B;
			double arcsin = asin(div);
			if (fabs(arcsin) < ZERO_THRESH)
				arcsin = 0.0;
			if (arcsin < 0.0)
				q1[0] = arcsin + 2.0*PI;
			else
				q1[0] = arcsin;
			q1[1] = PI - arcsin;
		}
		else if (fabs(B) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
				div = SIGN(d4)*SIGN(A);
			else
				div = d4 / A;
			double arccos = acos(div);
			q1[0] = arccos;
			q1[1] = 2.0*PI - arccos;
		}
		else if (d4*d4 > R) {
			return num_sols;
		}
		else {
			double arccos = acos(d4 / sqrt(R));
			double arctan = atan2(-B, A);
			double pos = arccos + arctan;
			double neg = -arccos + arctan;
			if (fabs(pos) < ZERO_THRESH)
				pos = 0.0;
			if (fabs(neg) < ZERO_THRESH)
				neg = 0.0;
			if (pos >= 0.0)
				q1[0] = pos;
			else
				q1[0] = 2.0*PI + pos;
			if (neg >= 0.0)
				q1[1] = neg;
			else
				q1[1] = 2.0*PI + neg;
		}
	}

	////////////////////////////// wrist 2 joint (q5) //////////////////////////////
	double q5[2][2];
	{
		for (int i = 0; i<2; i++) {
			double numer = (T03*sin(q1[i]) - T13*cos(q1[i]) - d4);
			double div;
			if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
				div = SIGN(numer) * SIGN(d6);
			else
				div = numer / d6;
			double arccos = acos(div);
			q5[i][0] = arccos;
			q5[i][1] = 2.0*PI - arccos;
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	{
		for (int i = 0; i<2; i++) {
			for (int j = 0; j<2; j++) {
				double c1 = cos(q1[i]), s1 = sin(q1[i]);
				double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
				double q6;
				////////////////////////////// wrist 3 joint (q6) //////////////////////////////
				if (fabs(s5) < ZERO_THRESH)
					q6 = q6_des;
				else {
					q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
						SIGN(s5)*(T00*s1 - T10*c1));
					if (fabs(q6) < ZERO_THRESH)
						q6 = 0.0;
					if (q6 < 0.0)
						q6 += 2.0*PI;
				}
				////////////////////////////////////////////////////////////////////////////////

				double q2[2], q3[2], q4[2];
				///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
				double c6 = cos(q6), s6 = sin(q6);
				double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
				double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
				double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
					T03*c1 + T13*s1;
				double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

				double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
				if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
					c3 = SIGN(c3);
				else if (fabs(c3) > 1.0) {
					// TODO NO SOLUTION
					continue;
				}
				double arccos = acos(c3);
				q3[0] = arccos;
				q3[1] = 2.0*PI - arccos;
				double denom = a2*a2 + a3*a3 + 2 * a2*a3*c3;
				double s3 = sin(arccos);
				double A = (a2 + a3*c3), B = a3*s3;
				q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
				q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
				double c23_0 = cos(q2[0] + q3[0]);
				double s23_0 = sin(q2[0] + q3[0]);
				double c23_1 = cos(q2[1] + q3[1]);
				double s23_1 = sin(q2[1] + q3[1]);
				q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
				q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
				////////////////////////////////////////////////////////////////////////////////
				for (int k = 0; k<2; k++) {
					if (fabs(q2[k]) < ZERO_THRESH)
						q2[k] = 0.0;
					else if (q2[k] < 0.0) q2[k] += 2.0*PI;
					if (fabs(q4[k]) < ZERO_THRESH)
						q4[k] = 0.0;
					else if (q4[k] < 0.0) q4[k] += 2.0*PI;
					ik_sols_[num_sols * 6 + 0] = q1[i];    ik_sols_[num_sols * 6 + 1] = q2[k];
					ik_sols_[num_sols * 6 + 2] = q3[k];    ik_sols_[num_sols * 6 + 3] = q4[k];
					ik_sols_[num_sols * 6 + 4] = q5[i][j]; ik_sols_[num_sols * 6 + 5] = q6;
					num_sols++;
				}
			}
		}
	}

#if 1
	// the solution joint angle may not be in the range we want
	for (int i = 0; i < num_sols; i++)
	{
		bool valid_solution = true;

		// try to bring the joint angle back to the range we want
		for (int j = 0; j < 6; j++)
		{
			double min = joint_range_[j * 2];
			double max = joint_range_[j * 2 + 1];

			double q = ik_sols_[i * 6 + j];

			if (q > max) q -= 2 * PI;
			else if (q < min) q += 2 * PI;
			else continue;

			if (q <= max && q >= min) ik_sols_[i * 6 + j] = q;
			else
			{
				valid_solution = false;
				break;
			}
		}

		if (valid_solution)
		{
			ik_sols_vec.push_back(i);
			/*	std::cout << ik_sols_vec.back() << ": ";
			for (int k = 0; k < 6; k++)
			std::cout << ik_sols_[i * 6 + k] << " ";
			std::cout << "\n";*/
		}
	}
#endif

	return num_sols;
}

void PathPlanner::double2float(double* array6_d, float* array6_f)
{
	for (int i = 0; i < 6; i++) array6_f[i] = array6_d[i];
}


void PathPlanner::PRMCEPreprocessing()
{
	// INITIALIZATION
	clock_t tic = clock();
	random_nodes_buffer_ = new float[num_nodes_ * num_joints_];

	// num_nodes*six joints*three coordinates(x,y,z)
	reference_points_buffer_ = new float[num_nodes_*num_ref_points_*3];

	start_end_ref_points_ = new float[2*3*num_ref_points_];

	// initGrid, z shift sign is different from those of x and y
	initGrid(128, 264, 200, 4, 0, 132, -80);	//growth chamber
	//initGrid(304, 128, 200, 4, 152, 0, -80);	//offset, base to grid
	clock_t toc = clock();
	printf("Init Grid Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	// GENERATE RANDOM SELF-COLLISION-FREE CONFIGURATIONS
	// this random configuration generation is not uniform due to non-independent randon numbers
	std::mt19937 rand_gen_;
	std::vector<std::uniform_real_distribution<float>> distri_vec_;
	distri_vec_.resize(6);
	// unit: rad, the joint range is different from the joint limit defined in the header file to reduce useless randon configurations
	std::uniform_real_distribution<float> d1(-220./180.*M_PI, -140./180.*M_PI);	// Base
	std::uniform_real_distribution<float> d2(-130./180.*M_PI, -80./180.*M_PI);	// Shoulder
	std::uniform_real_distribution<float> d3(-140./180.*M_PI, -40./180.*M_PI);	// Elbow
	std::uniform_real_distribution<float> d4(-70. / 180.*M_PI, 0. / 180.*M_PI);	// Wrist 1
	std::uniform_real_distribution<float> d5(80./180.*M_PI, 100./180.*M_PI);	// Wrist 2
	std::uniform_real_distribution<float> d6(-200./ 180.*M_PI, -160./ 180.*M_PI); // Wrist3

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
		for (int j = 0; j < num_joints_; j++) joint_array6[j] = distri_vec_[j](rand_gen_);

		// make wrist 3 (last joint) -180 to prevent cable twisting
		joint_array6[5] = -M_PI;

		if ( !selfCollision(joint_array6, true))
		{
			memcpy(random_nodes_buffer_ + i * num_joints_, joint_array6, num_joints_ * sizeof(float));
			//if (i % 100 == 0) std::cout << "random node " << i << "\n";
			i++;

		//	if (i > (num_nodes_*0.8)) tcp_y_limit_ = 0.6f;
		}
		else fail_count++;
	}

	// manually add regular poses, then inverse kinematics
	Eigen::Matrix4d tmp_hand_pose = Eigen::Matrix4d::Identity();

	bool overflow = false;
	int config_idx = 0;

	// for imaging
	config_idx = 0;
	tmp_hand_pose(0, 0) = 0.;
	tmp_hand_pose(1, 0) = -1.;
	tmp_hand_pose(1, 1) = 0.;
	tmp_hand_pose(0, 1) = -1.;
	tmp_hand_pose(2, 2) = -1.;

	for (float z = 0.2f; z <= 0.81f && !overflow; z += 0.1f)  //7
	{	
		for (float x = -0.85f; x <= -0.3f && !overflow; x += 0.1f) // 5
		{	
			for (float y = -0.6f; y <= 0.61f && !overflow; y += 0.1f)	// 13	
			{
				tmp_hand_pose(0, 3) = x;
				tmp_hand_pose(1, 3) = y;
				tmp_hand_pose(2, 3) = z;

				std::vector<int> ik_sols_vec;

				inverseKinematics(tmp_hand_pose, ik_sols_vec);

				for (auto idx : ik_sols_vec)
				{
					float sol_f[6];

					double2float(ik_sols_ + idx * num_joints_, sol_f);

					if (!selfCollision(sol_f)) {

						memcpy(random_nodes_buffer_ + config_idx * num_joints_, sol_f, num_joints_ * sizeof(float));

						config_idx++;

						if (config_idx >= num_nodes_) {

							overflow = true;
							break;
						}
					}
				}
			}
		}
	}

	std::cout << "successful manual configs for imaging: " << config_idx<< "\n";

	//manual poses that can bring the sensor head in and out of the chamber, do not clear config_idx
	if (config_idx < num_nodes_)
	{
		joint_array6[0] = -M_PI; joint_array6[4] = M_PI_2;	joint_array6[5] = -M_PI;
		joint_array6[1] = -67. / 180 * M_PI; joint_array6[2] = -121. / 180 * M_PI; joint_array6[3] = -5.4 / 180 * M_PI;
		
		memcpy(random_nodes_buffer_ + config_idx * num_joints_, joint_array6, num_joints_ * sizeof(float));
		++config_idx;
	}

	if (config_idx < num_nodes_)
	{
		joint_array6[1] = -52. / 180 * M_PI; joint_array6[2] = -130. / 180 * M_PI; joint_array6[3] = -3./ 180 * M_PI;
		memcpy(random_nodes_buffer_ + config_idx * num_joints_, joint_array6, num_joints_ * sizeof(float));
		++config_idx;
	}

	if (config_idx < num_nodes_)
	{
		joint_array6[1] = -10. / 180 * M_PI; joint_array6[2] = -138. / 180 * M_PI; joint_array6[3] = -28. / 180 * M_PI;
		memcpy(random_nodes_buffer_ + config_idx * num_joints_, joint_array6, num_joints_ * sizeof(float));
		++config_idx;
	}

	if (config_idx < num_nodes_)
	{
		joint_array6[1] = -1. / 180 * M_PI; joint_array6[2] = -141. / 180 * M_PI; joint_array6[3] = -36. / 180 * M_PI;
		memcpy(random_nodes_buffer_ + config_idx * num_joints_, joint_array6, num_joints_ * sizeof(float));
		++config_idx;
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

		tmp_random_nodes_buffer_ptr += num_joints_;
	}

	toc = clock();
	printf("referece points Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	// COMPUTE K NEAREST NEIGHBORS
	flann::Matrix<float> reference_points_mat = flann::Matrix<float>(reference_points_buffer_, num_nodes_, 3 * num_ref_points_);

	// L2 norm of reference points on arm in workspace
	referen_point_index_ = new flann::Index<flann::L2<float>>(reference_points_mat, flann::KDTreeIndexParams(4));	// number of parallel trees

	referen_point_index_->buildIndex();

	flann::Matrix<float> ref_p_query_mat = flann::Matrix<float>(reference_points_buffer_, num_nodes_, 3 * num_ref_points_);

	// neighbor index
	flann::Matrix<int> ref_p_indices_mat(new int[num_nodes_*ref_p_nn_], num_nodes_, ref_p_nn_);

	// distance 
	flann::Matrix<float> ref_p_dists_mat(new float[num_nodes_*ref_p_nn_], num_nodes_, ref_p_nn_);

	referen_point_index_->knnSearch(ref_p_query_mat, ref_p_indices_mat, ref_p_dists_mat, ref_p_nn_, flann::SearchParams(128));	//change 128 to CHECKS_UNLIMITED search all leaves

	toc = clock();
	printf("flann knn Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	// BUILD GRAPH
	// add edge based on radius search
	for (int node_id = 0; node_id < num_nodes_; node_id++)
	{
		//if (node_id % 50 == 0) std::cout << "add edge for node " << node_id << "\n";

		// add an edge between center node and neighbor node
		for (int n_idx = 1; n_idx < ref_p_nn_; n_idx++)	//n_idx = 0 is the node itself
		{
			int neighbor_id =  *(ref_p_indices_mat.ptr() + ref_p_nn_*node_id + n_idx);

			// actually squared cuz flann didn't sqrt
			float l2squared_workspace_dist = *(ref_p_dists_mat.ptr() + ref_p_nn_*node_id + n_idx);

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
				prmcegraph_[e].weight = l2squared_workspace_dist;
				prmcegraph_[e].weight_copy = l2squared_workspace_dist;
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
	const int num_edges = boost::num_edges(prmcegraph_);
	std::cout << "num of edges: " << num_edges << "\n";

	int counter = 0;
	tic = clock();
	// WORKSPACE MAPPING

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

		//viewOccupancyGrid();
		
		// round counter, reset occupancy grid status so that new edges can be added to cell
		prmce_round_counter_++;

		std::cout << ++counter << std::endl;
	}

	toc = clock();
	printf("map edges Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
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
	//printf("add point cloud Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
}

void PathPlanner::viewOccupancyGrid()
{
//	pcl::visualization::PCLVisualizer viewer;
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
					//p.x = ((x+0.5f)*cell_size_ + grid_offset_x_)*0.01f; p.y = ((y+0.5f)*cell_size_ + grid_offset_y_)*0.01f; p.z = ((z+0.5f)*cell_size_ + grid_offset_z_)*0.01f;
					p.x = (grid_offset_x_ - x*cell_size_ )*0.01f; p.y = (grid_offset_y_ - y*cell_size_)*0.01f; p.z = (z*cell_size_ + grid_offset_z_)*0.01f;
					p.r = p.g = p.b = 200;
					cloud->points.push_back(p);
					count++;
				}
			}
		}
	}

	std::cout << "vox cloud size: " << count/*cloud->points.size()*/ << std::endl;

//	viewer.removeAllPointClouds();
//	viewer.addPointCloud(cloud);
//	viewer.addCoordinateSystem(0.3, "world", 0);
//	viewer.spin();
}

bool PathPlanner::planPath(float* start_joint_pos, float* end_joint_pos, bool smooth, bool try_direct_path)
{
	if (!path_planner_ready_)
	{
		std::cout << "NO Path Planner Loaded!\n";
		return false;
	}

	clock_t tic = clock();
	//if (selfCollision(start_joint_pos))
	//{
	//	std::cout << "start config self collision \n";
	//	return false;
	//}

	if (selfCollision(end_joint_pos))
	{
		std::cout << "end config self collision \n";
		return false;
	}
	
	// check path from start to goal directly
	if (try_direct_path && !selfCollisionBetweenTwoConfigs(start_joint_pos, end_joint_pos) && !collisionCheckTwoConfigs(start_joint_pos, end_joint_pos))
	{
		shortest_path_index_vec_.clear();
		prmce_swept_volume_counter_++;
		return true;
	}
	clock_t toc = clock();
//	printf("Check direct path time: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	tic = clock();
	prmce_swept_volume_counter_++;
	
	std::vector<RefPoint> reference_points_start, reference_points_end;

	std::vector<Eigen::Matrix3f> rot_mats_start, rot_mats_end;

	computeReferencePointsOnArm(start_joint_pos, reference_points_start, rot_mats_start);

	computeReferencePointsOnArm(end_joint_pos, reference_points_end, rot_mats_end);

	for (int i = 0; i < num_ref_points_; i++) memcpy(start_end_ref_points_ + 3*i, reference_points_start[i].coordinates, 3 * sizeof(float));

	for (int i = 0; i < num_ref_points_; i++) memcpy(start_end_ref_points_ + 3*(num_ref_points_ + i), reference_points_end[i].coordinates, 3 * sizeof(float));

	flann::Matrix<float> query_mat = flann::Matrix<float>(start_end_ref_points_, 2, 3*num_ref_points_);

	// search the nearest neighbor
	const int num_neighbors = ref_p_nn_ * 4;
	flann::Matrix<int> indices_mat(new int[2 * num_neighbors], 2, num_neighbors);
	flann::Matrix<float> dists_mat(new float[2 * num_neighbors], 2, num_neighbors);

	// index_ for L2 in Cspace !
	referen_point_index_->knnSearch(query_mat, indices_mat, dists_mat, num_neighbors, flann::SearchParams(128));

	//for (int i = 0; i < 2 * ref_p_nn_; i++) std::cout << "index: " << *(indices_mat.ptr() + i) << " distance " << *(dists_mat.ptr() + i) << "\n";// << " cc label " << connected_component_[*(indices_mat.ptr() + i)] << "\n";

	int start = -1;
	int goal = -1;

	// check collision on path
	for (int i = 0; i < num_neighbors; i++)
	{
		int neighbor = *(indices_mat.ptr() + i);

		// collision check on edge
		if (!selfCollisionBetweenTwoConfigs(start_joint_pos, random_nodes_buffer_ + neighbor*num_joints_)
			//&& !collisionCheckTwoConfigs(start_joint_pos, random_nodes_buffer_ + neighbor*num_joints_)
			)
		{
			start = neighbor;
			break;
		}
		//else std::cout << "collision start with neighbor " << neighbor << "\n";
		prmce_swept_volume_counter_++;
	}

	prmce_swept_volume_counter_++;

	if (start == -1)
	{
		std::cout << "connection between start and roadmap fail \n";
		return false;
	}

	for (int i = 0; i < num_neighbors; i++)
	{
		int neighbor = *(indices_mat.ptr() + num_neighbors + i);

		if (!selfCollisionBetweenTwoConfigs(end_joint_pos, random_nodes_buffer_ + neighbor*num_joints_)
			//&& !collisionCheckTwoConfigs(end_joint_pos, random_nodes_buffer_ + neighbor*num_joints_)
			)
		{
			goal = neighbor;
			break;
		}
		//else std::cout << "collision goal with neighbor " << neighbor << "\n";
		prmce_swept_volume_counter_++;
	}

	prmce_swept_volume_counter_++;

	if (goal == -1)
	{
		std::cout << "connection between goal and roadmap fail \n";
		return false;
	}

#if 0
	toc = clock();
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
#endif

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
		//toc = clock();
		//printf("grahp search Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

		shortest_path_index_vec_.clear();

		for (prmcevertex_descriptor v = goal;; v = p[v])
		{
			shortest_path_index_vec_.push_back(v);
			if (p[v] == v) break;
		}

		std::reverse(std::begin(shortest_path_index_vec_), std::end(shortest_path_index_vec_));

#if 0
		std::cout << "A* shortest path from " << start << " to " << goal  << ": ";
		std::vector<prmcevertex_descriptor>::iterator spi = shortest_path_index_vec_.begin();
		for (++spi; spi != shortest_path_index_vec_.end(); ++spi) std::cout << " -> " << *spi;
		std::cout << std::endl << "Total travel distance (m): " << sqrt(d[goal]) << std::endl;
#endif

		found_path = d[goal] < 500.f ? true : false;
	}
		
	if (smooth)
	{
		std::cout << "Smoothing path...\n";
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
	float* inter_joint_pos= new float[num_joints_];

	prmce_collision_found_ = false;

	float max_dist = 0.f;

	for (int i = 0; i < num_joints_; i++)
	{
		float dist = std::abs(config1[i] - config2[i]);

		if (dist > max_dist)
		{
			max_dist = dist;
		}
	}

	float min_layer_alpha = 1.74e-4f / max_dist;

	min_layer_alpha = std::min(min_layer_alpha, 0.3f);

	// bisection style
	for (float layer_alpha = 0.5f; layer_alpha > min_layer_alpha; layer_alpha *= 0.5f)
	{
		int num_voxelized = 0;

		for (float alpha = layer_alpha; alpha < 1.0f; alpha += 2.f*layer_alpha)
		{
			for (int i = 0; i < num_joints_; i++)	inter_joint_pos[i] = (alpha*config1[i] + (1.0f - alpha)*config2[i]);

			std::vector<RefPoint> reference_points;
			std::vector<Eigen::Matrix3f> rot_mats;
			std::vector<OBB> arm_obbs;
			std::vector<prmceedge_descriptor> edge_vec;

			computeReferencePointsOnArm(inter_joint_pos, reference_points, rot_mats);

			getArmOBBModel(reference_points, rot_mats, arm_obbs);

			num_voxelized += voxelizeArmConfig(arm_obbs, edge_vec, true);

			if (prmce_collision_found_)
			{
				prmce_swept_volume_counter_++;
				return true;
			}
		}

		//std::cout << "layer " << layer_alpha << " num vox: " << num_voxelized << "\n";

		//if (num_voxelized == 0)	break;
	}

	prmce_swept_volume_counter_++;
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

	if (boost::filesystem::exists(filename+".dat") 
		&& boost::filesystem::exists(filename + ".idx")
		)
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
	for (int i = 0; i < num_cells_; i++)
	{
		grid_[i].isOccupiedCounter = 0;
		grid_[i].sweptVolumneCounter = 0;
	}

	prmce_round_counter_ = prmce_swept_volume_counter_ = 1;
}

void PathPlanner::smoothPath()
{
	if (shortest_path_index_vec_.size() <= 1) return;

	clock_t tic = clock();

	if (shortest_path_index_vec_.size() == 2)
	{
		if (!collisionCheckTwoConfigs(random_nodes_buffer_ + shortest_path_index_vec_[0] * num_joints_,
			random_nodes_buffer_ + shortest_path_index_vec_[1] * num_joints_)
		 && !selfCollisionBetweenTwoConfigs(random_nodes_buffer_ + shortest_path_index_vec_[0] * num_joints_,
			random_nodes_buffer_ + shortest_path_index_vec_[1] * num_joints_)
			)
		{
			shortest_path_index_vec_.pop_back();
		}
#if 0
		clock_t toc = clock();
		printf("smooth path Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

		std::cout << "smooth path: ";
		for (auto e : shortest_path_index_vec_) std::cout << e << "->";
		std::cout << "\n";
#endif
		prmce_swept_volume_counter_++;
		return;
	}

	int cur_vertex = 0;
	int new_vertex = 2;
	
	std::vector<prmcevertex_descriptor> smooth_path;
	smooth_path.push_back(shortest_path_index_vec_[0]);

	while (new_vertex < shortest_path_index_vec_.size())
	{
		while (new_vertex < shortest_path_index_vec_.size() &&
			!collisionCheckTwoConfigs(random_nodes_buffer_ + shortest_path_index_vec_[cur_vertex]*num_joints_, 
										random_nodes_buffer_ + shortest_path_index_vec_[new_vertex]*num_joints_)
		 && !selfCollisionBetweenTwoConfigs(random_nodes_buffer_ + shortest_path_index_vec_[cur_vertex] * num_joints_,
			 random_nodes_buffer_ + shortest_path_index_vec_[new_vertex] * num_joints_)
			)
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
//	printf("smooth path Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

//	std::cout << "smooth path: ";
//	for (auto e : smooth_path) std::cout << e << "->";
//	std::cout << "\n";

	shortest_path_index_vec_.swap(smooth_path);
}

// does not have a shorten_probe flag because "moveToConfigGetKinectPointCloud" could be used for both imaging and probing
bool PathPlanner::collisionCheckForSingleConfig(float* config, bool shorten_probe)
{
	if (selfCollision(config))
	{
		//std::cout << "self collision\n";
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

	int num_voxelized = voxelizeArmConfig(arm_obbs, edge_vec, true, shorten_probe);

	prmce_swept_volume_counter_++;

	return prmce_collision_found_;
}