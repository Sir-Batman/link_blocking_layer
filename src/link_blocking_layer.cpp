#include <link_blocking_layer/link_blocking_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(link_blocking_namespace::BlockingLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace link_blocking_namespace
{
	BlockingLayer::BlockingLayer() {};

	void BlockingLayer::onInitialize()
	{
		//points.push_back(std::pair<double, double>(-3,0));
		points.push_back(std::pair<double, double>(3,-1));
		points.push_back(std::pair<double, double>(6,-1));
		//points.push_back(std::pair<double, double>(0,0));
		counter_ = 0;

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

		int r;
		while (to_add.size() > 0)
		{
			r = addWall(to_add[0], min_x, min_y, max_x, max_y);
			current_blocks.push_back(to_add[0]);
			to_add.erase(to_add.begin());
		}
		while (to_remove.size() > 0)
		{
			r = removeWall(to_remove[0], min_x, min_y, max_x, max_y);
			// Find and remove from current_blocks
			for (int i = 0; i < current_blocks.size(); ++i)
			{
				if (current_blocks[i].first.first == to_remove[0].first.first &&
						current_blocks[i].first.second == to_remove[0].first.second &&
						current_blocks[i].second.first == to_remove[0].second.first &&
						current_blocks[i].second.second == to_remove[0].second.second)
				{
					current_blocks.erase(current_blocks.begin() + i);
					break;
				}
			}
			to_remove.erase(to_remove.begin());
		}

	}

	int BlockingLayer::removeWall(wall &w,double* min_x, double* min_y, double* max_x, double* max_y) 
	{
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
				setCost(x, y, FREE_SPACE);
			}
		}
		else 
		{
			for (x= p_1.first; x != p_2.first; (p_1.first < p_2.first ? ++x : --x))
			{
				y = p_1.second + dy*(x + p_1.first)/dx;
				setCost(x, y, FREE_SPACE);
			}
		}
		// Update the max and min boundaries, potentially from point i or i+1
		//ROS_INFO("radius: %lf", cell_inflation_radius_);
		*min_x = std::min(*min_x, w.first.first-1.5);
		*min_y = std::min(*min_y, w.first.second-1.5);
		*max_x = std::max(*max_x, w.first.first+1.5);
		*max_y = std::max(*max_y, w.first.second+1.5);

		*min_x = std::min(*min_x, w.second.first-1.5);
		*min_y = std::min(*min_y, w.second.second-1.5);
		*max_x = std::max(*max_x, w.second.first+1.5);
		*max_y = std::max(*max_y, w.second.second+1.5);

		return 0;
	}// End removeWall()

	/* This adds the wall to the costmap. Assumes that the wall is already stored in 
	 * class list of walls being kept track of. */
	int BlockingLayer::addWall(wall &w, double* min_x, double* min_y, double* max_x, double* max_y)
	{
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

	/* Takes in 4 floating point numbers, [p1X, p1Y, p2X, p2Y]
	 * and constructs a wall between the points by converting 
	 * to the wall format and adding the wall to the to_add list */
	void BlockingLayer::convertAndAdd(float points[])
	{
		wall w;
		w.first.first   = points[0];
		w.first.second  = points[1];
		w.second.first  = points[2];
		w.second.second = points[3];
		to_add.push_back(w);
	}

	/* Takes in 4 floating point numbers, [p1X, p1Y, p2X, p2Y]
	 * and constructs a wall between the points by converting 
	 * to the wall format and adding the wall to the to_remove list */
	void BlockingLayer::convertAndRemove(float points[])
	{
		wall w;
		w.first.first   = points[0];
		w.first.second  = points[1];
		w.second.first  = points[2];
		w.second.second = points[3];
		to_remove.push_back(w);
	}

	/* Simple method to clean out all the walls currently being used */
	void BlockingLayer::clearWalls()
	{
		for (int i = 0; i < current_blocks.size(); ++i)
		{
			to_remove.push_back(current_blocks[i]);
		}
	}

} // end namespace
