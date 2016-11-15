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

	flann::Index<flann::L2<float>>* referen_point_index_;

	std::mt19937 rand_gen_;

	std::vector<std::uniform_real_distribution<float>> distri_vec_;

	// DH parameters for UR10
	float a_[6];
	float d_[6];
	float alpha_[6];

	Eigen::Matrix4f DH_mat_vec_[6];
	Eigen::Matrix4f fk_mat_;	//transformation of last frame

	pcl::octree::OctreePointCloudSearch<PointT> octree_;
	
	std::vector<PointT> arm_joint_points_;

	std::vector<std::vector<PointT>> collision_vec_;

	prmcegraph_t prmcegraph_;

	int num_ref_points_;

	int ref_p_nn_;

	int num_joints_;

	std::vector<int> connected_component_;

	std::vector<prmcevertex_descriptor> shortest_path_index_vec_;

	float arm_radius_;

	// 3d occupancy grid, unit: cm
	int grid_width_, grid_depth_, grid_height_, cell_size_, num_cells_;
	int grid_width_cm_, grid_depth_cm_, grid_height_cm_;
	int grid_offset_x_, grid_offset_y_, grid_offset_z_;	// UR10 base frame, local to world translation
	std::vector<Cell> grid_;

	float arm_radius_lookup[8] = {0.08f, 0.08f, 0.06f, 0.06f, 0.045f, 0.045f, 0.045f, 0.045f};

	// UR10 joint range
	double joint_range_[12] = { -200./180.*M_PI, 20./180.*M_PI,	// base
								-180./180.*M_PI, 0./180.*M_PI,	// shoulder
								-160.f/180.f*M_PI, -10.f/180.f*M_PI,	// elbow
								-170./180.*M_PI, 10./180.* M_PI,	// wrist 1
								10.f/180.f*M_PI, 170.f/180.f*M_PI,	// wrist 2
								-210.f/180.f*M_PI, -170.f/180.f*M_PI // wrist 3
								};

	int prmce_round_counter_;

	int prmce_swept_volume_counter_;

	bool prmce_collision_found_;	// used for collision check via swept volume

	bool path_planner_ready_;

	const int start_check_obb_idx_ = 5;	

	PathPlanner();

	~PathPlanner();

	Eigen::Matrix4f constructDHMatrix(int target_joint_id, float target_joint_pos);

	void forwardKinematicsUR10(float* joint_array6);

	bool collisionCheck(float* joint_array6, float radius);

	bool checkCollisionBetweenTwoConfig(float* center_config, float* neighbor_config, float dist, float step_size);

	int initGrid(int width, int depth, int height, int cell_size, int offset_x, int offset_y, int offset_z);

	void blockCells(PointT & point_in_arm_base);

	bool addEdgeDescriptor2Cell(std::vector<prmceedge_descriptor> & edge_vec, int cell_index);

	void computeReferencePointsOnArm(float* joint_pos, std::vector<RefPoint> & reference_points, std::vector<Eigen::Matrix3f> & rot_mats);

	// oriented bounding box collision
	bool collisionOBB(OBB & obb0, OBB & obb1);

	void constructOBB(RefPoint & p0, RefPoint & p1, Eigen::Matrix3f rot, float radius, int axis, OBB & obb);

	void getArmOBBModel(std::vector<RefPoint> ref_points, std::vector<Eigen::Matrix3f> & rot_mats, std::vector<OBB> & arm_obbs);

	bool selfCollision(float* joint_pos);

	// resolution in meter, return number of voxels processed
	int voxelizeLine(RefPoint & p1, RefPoint & p2, std::vector<RefPoint> & saved_points_grid_frame, std::vector<prmceedge_descriptor> & edge_vec, 
						bool point_in_robot_base_frame, bool save_points, bool set_occupy, bool set_swept_volume);

	int voxelizeOBB(OBB & obb, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume);

	int voxelizeArmConfig(std::vector<OBB> & arm_obbs, std::vector<prmceedge_descriptor> & edge_vec, bool set_swept_volume);

	void sweptVolume(float* joint_pos1, float* joint_pos2, std::vector<prmceedge_descriptor> & edge_vec);

	bool selfCollisionBetweenTwoConfigs(float* config1, float* config2);

	void PRMCEPreprocessing();

	void addPointCloudToOccupancyGrid(PointCloudT::Ptr cloud);

	void viewOccupancyGrid(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

	bool planPath(float* start_joint_pos, float* end_joint_pos, bool smooth, bool try_direct_path);

	bool collisionCheckTwoConfigs(float* config1, float* config2);

	//serialization
	void savePathPlanner(std::string filename);

	bool loadPathPlanner(std::string filename);

	void resetOccupancyGrid();

	void smoothPath();

	bool collisionCheckForSingleConfig(float* config);
};

#endif
