#ifndef LINK_BLOCKING_LAYER_H_
#define LINK_BLOCKING_LAYER_H_
#include <ros/ros.h>
#include <ros/console.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <vector>

namespace link_blocking_namespace
{
	typedef std::pair<double, double> point;
	typedef std::pair<point, point> wall;

	class BlockingLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
	{
		public:
			BlockingLayer();

			virtual void onInitialize();
			virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
			virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
			bool isDiscretized()
			{
				return true;
			}

			virtual void matchSize();
			void convertAndAdd(float points[]);
			void convertAndRemove(float points[]);
			void clearWalls();

		private:
			// Members
			std::vector<wall> current_blocks; // Keeps track of the currently implemented walls
			std::vector<wall> to_add; // List of walls to add, queried by updateBounds()
			std::vector<wall> to_remove; // List of walls to remvoe, queried by updateBounds();
			boost::recursive_mutex lock_;
			std::vector<point> points; // Deprecated
			int counter_; // Deprecated, purely for testing purposes

			void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
			dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
			int addWall(wall &w, double* min_x, double* min_y, double* max_x, double* max_y);
			int removeWall(wall &w, double* min_x, double* min_y, double* max_x, double* max_y);
	};
}
#endif
