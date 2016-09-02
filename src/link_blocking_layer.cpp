#include <link_blocking_layer/link_blocking_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(link_blocking_namespace::BlockingLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace link_blocking_namespace
{
	BlockingLayer::BlockingLayer() {};

	void BlockingLayer::onInitialize()
	{
		points.push_back(std::pair<double, double>(-3,0));
		points.push_back(std::pair<double, double>(-3,-1));
		points.push_back(std::pair<double, double>(0,-1));
		//points.push_back(std::pair<double, double>(0,0));

		ros::NodeHandle nh("~/" + name_);
		current_ = true;
		default_value_ = NO_INFORMATION;
		matchSize();

		dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind( &BlockingLayer::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}

	void BlockingLayer::matchSize()
	{
		Costmap2D* master = layered_costmap_->getCostmap();
		resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
	}


	void BlockingLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
	}

	/* updateBounds polls the various available lists of walls to modify.
	 * Depending on the presence of values in these lists, it makes calls
	 * to the appropriate priavte functions to add and subtract walls from
	 * the costmap. */
	void BlockingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
	{
		boost::recursive_mutex::scoped_lock lock(lock_);
		if (!enabled_) { return; }

		if (points.size() == 0) { return; }
		wall w;
		w.first = points[0];
		w.second = points[1];
		int r = addWall(w, min_x, min_y, max_x, max_y);

		// Remove the first two points now that they are marked
		points.erase(points.begin(),points.begin()+1);
	}

	int BlockingLayer::removeWall(wall &w,double* min_x, double* min_y, double* max_x, double* max_y) {return 0;}

	/* This adds the wall to the costmap. Assumes that the wall is already stored in 
	 * class list of walls being kept track of. */
	int BlockingLayer::addWall(wall &w, double* min_x, double* min_y, double* max_x, double* max_y)
	{
		// Create 1 line at a time, instead of trying to poplate all available
		// lines at once, so there's no major lag/bottleneck in waiting for all
		// lines to be created at once before the first one is available to be
		// rendered 
		// Convert points 0 and 1 stored in wall w from world coordinates to map coordinates
		unsigned int mx = 0;
		unsigned int my = 0;
		std::pair<unsigned int, unsigned int> p_1;
		std::pair<unsigned int, unsigned int> p_2;

		if(worldToMap(w.first.first, w.first.second, mx, my))
		{
			p_1.first  = mx;
			p_1.second = my;
		}
		else
		{
			//ROS_INFO("ERROR CONVERTING p1(%lf, %lf)",w.first.first, w.first.second);
			return -1;
		}
		if(worldToMap(w.second.first, w.second.second, mx, my))
		{
			p_2.first  = mx;
			p_2.second = my;
		}
		else
		{
			//ROS_INFO("ERROR CONVERTING p2(%lf, %lf)", w.second.first, w.second.second);
			return -1;
		}

		// Mark the rasterized line between the points i and i+1 as deadly
		unsigned int dx = p_2.first - p_1.first;
		unsigned int dy = p_2.second - p_1.second;
		unsigned int y = 0;
		unsigned int x = 0;
		// Special case for 'vertical' lines
		if (dx == 0)
		{
			x = p_1.first;
			for (y = p_1.second; y != p_2.second; (p_1.second < p_2.second ? ++y : --y))
			{
				setCost(x, y, LETHAL_OBSTACLE);
			}
		}
		else 
		{
			for (x= p_1.first; x != p_2.first; (p_1.first < p_2.first ? ++x : --x))
			{
				y = p_1.second + dy*(x + p_1.first)/dx;
				setCost(x, y, LETHAL_OBSTACLE);
			}
		}
		// Update the max and min boundaries, potentially from point i or i+1
		*min_x = std::min(*min_x, w.first.first);
		*min_y = std::min(*min_y, w.first.second);
		*max_x = std::max(*max_x, w.first.first);
		*max_y = std::max(*max_y, w.first.second);

		*min_x = std::min(*min_x, w.second.first);
		*min_y = std::min(*min_y, w.second.second);
		*max_x = std::max(*max_x, w.second.first);
		*max_y = std::max(*max_y, w.second.second);

		return 0;
	}// End addWall()


	/* Writes any new cost updates to the global costmap */
	void BlockingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
	{
		//ROS_INFO("<BLOCKING LAYER> In updateCosts");
		//ROS_INFO("    min: (%d, %d)    max: (%d, %d)", min_i, min_j, max_i, max_j);
		if (!enabled_) { return; }

		for (int j = min_j; j < max_j; j++)
		{
			for (int i = min_i; i < max_i; i++)
			{
				int index = getIndex(i, j);
				if (costmap_[index] == NO_INFORMATION)
				{
					continue;
				}
				master_grid.setCost(i, j, costmap_[index]); 
			}
		}
	}

} // end namespace
