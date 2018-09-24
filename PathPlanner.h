#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <random>
#include <flann/flann.hpp>
#include <Eigen/Core>
#include <math.h>
#include <time.h>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/random.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/filesystem.hpp>

// serialization
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <fstream>

struct PathPlanner
{
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

	struct PRMCEEdge
	{
		float weight;
		float weight_copy;

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) { ar & weight; ar & weight_copy; }
	};

	// ; use setS, no parallel edge
	typedef boost::adjacency_list < boost::hash_setS, boost::vecS, boost::undirectedS, 
									boost::no_property,	PRMCEEdge> prmcegraph_t;
	typedef prmcegraph_t::vertex_descriptor prmcevertex_descriptor;
	typedef prmcegraph_t::edge_descriptor prmceedge_descriptor;
	typedef std::pair<int, int> prmceedge;

	struct Cell
	{
		int isOccupiedCounter;
		int sweptVolumneCounter;
		//std::vector<prmceedge_descriptor> edges;
		std::vector<std::pair<int, int>> edges;

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & isOccupiedCounter;
			ar & sweptVolumneCounter;
			ar & edges;
		}
	};

	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive & ar, const unsigned int version) const
	{
		ar & grid_width_; ar & grid_depth_; ar & grid_height_;
		ar & grid_width_cm_; ar & grid_depth_cm_; ar & grid_height_cm_;
		ar & cell_size_; ar & num_cells_; ar & num_joints_; ar & prmce_round_counter_;
		ar & grid_offset_x_; ar & grid_offset_y_; ar & grid_offset_z_;
		ar & num_nodes_; ar & ref_p_nn_; ar & num_ref_points_;
		ar & prmcegraph_;

		for (int i = 0; i < num_nodes_ * 6; i++)
			ar & random_nodes_buffer_[i];

		for (int i = 0; i < num_nodes_ * 3 * num_ref_points_; i++)
			ar & reference_points_buffer_[i];

		for (int i = 0; i < num_cells_; i++)
			ar & grid_[i];
	}

	template<class Archive>
	void load(Archive & ar, const unsigned int version)
	{
		ar & grid_width_; ar & grid_depth_; ar & grid_height_;
		ar & grid_width_cm_; ar & grid_depth_cm_; ar & grid_height_cm_;
		ar & cell_size_; ar & num_cells_; ar & num_joints_; ar & prmce_round_counter_;
		ar & grid_offset_x_; ar & grid_offset_y_; ar & grid_offset_z_;
		ar & num_nodes_; ar & ref_p_nn_; ar & num_ref_points_;
		ar & prmcegraph_;

		random_nodes_buffer_ = new float[num_nodes_ * 6];
		reference_points_buffer_ = new float[num_nodes_ * 3 * num_ref_points_];
		grid_.clear(); grid_.resize(num_cells_);

		for (int i = 0; i < num_nodes_ * 6; i++)
			ar & random_nodes_buffer_[i];

		for (int i = 0; i < num_nodes_ * 3 * num_ref_points_; i++)
			ar & reference_points_buffer_[i];

		for (int i = 0; i < num_cells_; i++)
			ar & grid_[i];
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()

	// A* euclidean distance heuristic
	template <class Graph, class CostType, class LocMap, class Pitch>
	class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
	{
	public:
		typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
		distance_heuristic(LocMap l, Vertex goal, Pitch pitch)
			: m_location(l), m_goal(goal), m_pitch(pitch) {}
		CostType operator()(Vertex u)
		{
			CostType L2_norm = 0;
			for (int i = 0; i < m_pitch; i++)
			{
				CostType dif = m_location[m_goal*m_pitch + i] - m_location[u*m_pitch + i];
				L2_norm += dif*dif;
			}
			return L2_norm;
		}
	private:
		LocMap m_location;
		Vertex m_goal;
		Pitch m_pitch;
	};

	// visitor that terminates when goal found
	template<class Vertex>
	class astar_goal_visitor : public boost::default_astar_visitor
	{
	public:
		astar_goal_visitor(Vertex goal) : m_goal(goal) {}
		template <class Graph>
		void examine_vertex(Vertex u, Graph& g) {
			if (u == m_goal)
				throw found_goal();
		}
	private:
		Vertex m_goal;
	};

	struct ArmConfig
	{
		float joint_pos_array[6];
	};

	struct RefPoint
	{
		float coordinates[3];
	};

	struct found_goal {};	// exception for A star termination

	// oriented bounding box
	struct OBB
	{
		Eigen::Vector3f C;	// center
		Eigen::Matrix3f A;	// right hand orthonormal axes, rotation matrix
		Eigen::Vector3f a;	//extents, positive
	};

	int num_nodes_;

	float* random_nodes_buffer_ = NULL;

	// reference points on the robot arm
	float* reference_points_buffer_ = NULL;

	float* start_end_ref_points_ = NULL;

	flann::Index<flann::L2<float>>* referen_point_index_; // first half for probing

	std::mt19937 rand_gen_;

	std::vector<std::uniform_real_distribution<float>> distri_vec_;

	// DH parameters for UR10
	float a_[6];
	float d_[6];
	float alpha_[6];

	Eigen::Matrix4f DH_mat_vec_[6];
	Eigen::Matrix4f fk_mat_;	//transformation of last frame

	std::vector<PointT> arm_joint_points_;

	std::vector<std::vector<PointT>> collision_vec_;

	prmcegraph_t prmcegraph_;

	int num_ref_points_;

	int ref_p_nn_;

	int num_joints_;

	std::vector<int> connected_component_;

	std::vector<prmcevertex_descriptor> shortest_path_index_vec_;

	// 3d occupancy grid, unit: cm
	int grid_width_, grid_depth_, grid_height_, cell_size_, num_cells_;
	int grid_width_cm_, grid_depth_cm_, grid_height_cm_;
	int grid_offset_x_, grid_offset_y_, grid_offset_z_;	// UR10 base frame, local to world translation
	std::vector<Cell> grid_;

	float arm_radius_lookup[8] = { 0.08f, 0.08f, 0.06f, 0.06f, 0.045f, 0.045f, 0.045f, 0.045f };

	// UR10 joint range
	double joint_range_[12] = { -200./180.*M_PI, 20./180.*M_PI,	// base
								-180./180.*M_PI, 0./180.*M_PI,	// shoulder
								-160./180.f*M_PI, -10./180.f*M_PI,	// elbow
								-170./180.*M_PI, 10./180.* M_PI,	// wrist 1
								10.f/180.f*M_PI, 170.f/180.f*M_PI,	// wrist 2
								//-250. / 180.f*M_PI, -90. / 180.f*M_PI // wrist 3
								-270. / 180.f*M_PI, -70. / 180.f*M_PI // wrist 3
								};

	double probing_joint_range_wrist_2_[2] = {-130./180.*M_PI, -50./180.*M_PI};	// when probing, wrist 2 needs to be rotated by -180 first

	int prmce_round_counter_;

	int prmce_swept_volume_counter_;

	bool prmce_collision_found_;	// used for collision check via swept volume

	bool path_planner_ready_;

	const int start_check_obb_idx_ = 7;
	const int end_check_obb_idx_ = 10;

	float tcp_y_limit_ = -0.3f;

	double ik_sols_[8 * 6];

	// UR10 dh parameters
	const double d1 = 0.1273;
	const double a2 = -0.612;
	const double a3 = -0.5723;
	const double d4 = 0.163941;
	const double d5 = 0.1157;
	const double d6 = 0.0922;

	const double ZERO_THRESH = 1e-10;
	int SIGN(double x) { return (x > 0) - (x < 0); }
	const double PI = M_PI;

	double probe_position[3] = { 0.0215, 0.1275, -0.3}; // in robot hand pose
	double cylinder_back_position[3] = { 0.025, 0.13, -0.33 }; // in robot hand pose

	PathPlanner();

	~PathPlanner();

	Eigen::Matrix4f constructDHMatrix(int target_joint_id, float target_joint_pos);

	void forwardKinematicsUR10(float* joint_array6);
	void forwardKinematicsUR10ROS(float* joint_array6);

	int initGrid(int width, int depth, int height, int cell_size, int offset_x, int offset_y, int offset_z);

	void blockCells(PointT & point_in_arm_base);

	bool addEdgeDescriptor2Cell(std::vector<prmceedge_descriptor> & edge_vec, int cell_index);

	void computeReferencePointsOnArm(float* joint_pos, std::vector<RefPoint> & reference_points, std::vector<Eigen::Matrix3f> & rot_mats);

	// oriented bounding box collision
	bool collisionOBB(OBB & obb0, OBB & obb1);

	void constructOBB(RefPoint & p0, RefPoint & p1, Eigen::Matrix3f rot, float radius, int axis, OBB & obb);

	void getArmOBBModel(std::vector<RefPoint> ref_points, std::vector<Eigen::Matrix3f> & rot_mats, std::vector<OBB> & arm_obbs);

	bool selfCollision(float* joint_pos, bool pre_processing_stage=false);

	// resolution in meter, return number of voxels processed
	int voxelizeLine(RefPoint & p1, RefPoint & p2, std::vector<RefPoint> & saved_points_grid_frame, std::vector<prmceedge_descriptor> & edge_vec, 
						bool point_in_robot_base_frame, bool save_points, bool set_occupy, bool set_swept_volume);

	int voxelizeOBB(OBB & obb, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume);

	int voxelizeArmConfig(std::vector<OBB> & arm_obbs, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume, bool shorten_probe);

	void sweptVolume(float* joint_pos1, float* joint_pos2, std::vector<prmceedge_descriptor> & edge_vec);

	bool selfCollisionBetweenTwoConfigs(float* config1, float* config2);

	int inverseKinematics(Eigen::Matrix4d & T, std::vector<int> & ik_sols_vec);
	
	void double2float(double* array6_d, float* array6_f);

	void PRMCEPreprocessing();

	void addPointCloudToOccupancyGrid(PointCloudT::Ptr cloud);

	void viewOccupancyGrid();

	bool planPath(float* start_joint_pos, float* end_joint_pos, bool smooth = false, bool try_direct_path = true);

	bool collisionCheckTwoConfigs(float* config1, float* config2);

	//serialization
	void savePathPlanner(std::string filename);

	bool loadPathPlanner(std::string filename);

	void resetOccupancyGrid();

	void smoothPath();

	bool collisionCheckForSingleConfig(float* config, bool shorten_probe = false);
};

#endif
