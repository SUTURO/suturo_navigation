#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <object_finder/ObjectFinderConfig.h>
#include <tf/transform_listener.h>
#include <navigation_msgs/Area.h>

#include <mutex>
#include <cmath>

/**
 * Simple struct to save a x,y position with an index (i)
 */
struct pos {
    unsigned int x;
    unsigned int y;
    unsigned int i;
};

/**
 * Calculates the euclidean distance between two points on the xy-area
 * @param a Point a
 * @param b Point b
 * @return Euclidean distance between a and b on the xy-area
 */
static double get_dist_xy(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    return sqrt(pow(abs(a.x - b.x), 2) + pow(abs(a.y - b.y), 2));
}

/**
 * Calculates the angle between the two vectors (points) on the xy-area
 * @param a
 * @param b
 * @return angle between the two vectors (points) on the xy-area
 */
static double get_angle_xy(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    return atan2(a.y - b.y, a.x - b.x);
}

namespace object_finder {

    /**
     * Searches a costmap for connected occupied areas and publishes their poses in a pose_array. The areas can further
     * be constrained by size and probability accumulated over time.
     */
    class ObjectFinder {
    public:
        ObjectFinder(tf2_ros::Buffer &tf) {
            ros::NodeHandle nh("~");
            costmap_ = std::make_shared<costmap_2d::Costmap2DROS>("costmap", tf);
            object_publisher_ = nh.advertise<geometry_msgs::PoseArray>("object_finder", 10);
            ros::param::param<std::string>("global_frame", global_frame_, "map");
            ros::param::param<std::string>("robot_base_frame", robot_base_frame_, "base_footprint");
            ros::param::param<int>("frequency", frequency_, 1);
            ros::param::param<bool>("use_probability", use_probability_, true);
            ros::param::param<int>("min_confidence", min_confidence_, 50);
            ros::param::param<int>("p_obj", p_obj_, 70);
            ros::param::param<int>("p_free", p_free_, 40);
            ros::param::param<double>("min_angle", min_angle_, -0.52);
            ros::param::param<double>("max_angle", max_angle_, 0.52);
            ros::param::param<double>("max_update_range", max_update_range_, 3.0);
            ros::param::param<double>("min_update_range", min_update_range_, 1.0);
            ros::param::param<double>("max_dist_error", max_dist_error_, 0.05);
            dsrv_ = new dynamic_reconfigure::Server<ObjectFinderConfig>(nh);
            dynamic_reconfigure::Server<ObjectFinderConfig>::CallbackType cb = boost::bind(
                    &ObjectFinder::reconfigure_callback, this, _1, _2);
            dsrv_->setCallback(cb);
            ignore_areas_sub_ = nh.subscribe("ignore_area", 10, &ObjectFinder::ignore_area_callback, this);
        }

        ~ObjectFinder()
        {
            if (dsrv_)
                delete dsrv_;
        }

        /**
         * Starts the object finder
         */
        void run() {
            objects_.clear();
            confidence_.clear();
            ros::Rate loop_rate(frequency_);
            while (ros::ok()) {
                std::vector<geometry_msgs::Pose> found_objects = get_objects();
                // with probability enabled
                if(use_probability_){
                    // poses to publish
                    std::vector<geometry_msgs::Pose> p_objects;
                    update_confidence(found_objects);
                    int i = 0;
                    // filter by confidence
                    for(auto conf : confidence_){
                        ROS_INFO_STREAM("Object: " << i << " confidence: " << conf << " min_confidence: " << min_confidence_);
                        if(conf >= min_confidence_){
                            p_objects.push_back(objects_[i]);
                        }
                        i++;
                    }
                    publish_objects(filter_by_ignored_areas(p_objects));
                // without probability
                } else{
                    publish_objects(filter_by_ignored_areas(found_objects));
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
        std::vector<geometry_msgs::Pose> objects_; // poses of found objects
        std::vector<int> confidence_; // confidence of found objects (matched by index)
        ros::Publisher object_publisher_;
        int max_object_size_, min_object_size_, threshold_occupied_, frequency_, min_confidence_, p_obj_, p_free_;
        dynamic_reconfigure::Server<ObjectFinderConfig> *dsrv_;
        bool use_probability_;
        double min_angle_, max_angle_, max_update_range_, min_update_range_, max_dist_error_;
        tf::TransformListener tf_listener_;
        std::string robot_base_frame_, global_frame_;
        std::mutex ignore_areas_mutex_;
        std::vector<navigation_msgs::Area> ignore_areas_;
        ros::Subscriber ignore_areas_sub_;


        void ignore_area_callback(const navigation_msgs::AreaConstPtr& msg) {
            ignore_areas_mutex_.lock();
                ignore_areas_.push_back(*msg);
            ignore_areas_mutex_.unlock();
        }

        /**
         * Callback function for dynamic reconfigure
         * @param config
         * @param level
         */
        void reconfigure_callback(ObjectFinderConfig &config, uint32_t level) {
            ROS_INFO_STREAM("Reconfiguring");
            min_object_size_ = config.min_object_size;
            max_object_size_ = config.max_object_size;
            threshold_occupied_ = config.threshold_occupied;
            min_confidence_ = config.min_confidence;
            p_obj_ = config.p_obj;
            p_free_ = config.p_free;
            min_angle_ = config.min_angle;
            max_angle_ = config.max_angle;
            max_update_range_ = config.max_update_range;
            min_update_range_ = config.min_update_range;
            max_dist_error_ = config.max_dist_error;
            use_probability_ = config.use_probability;
        }

        std::vector<geometry_msgs::Pose> filter_by_ignored_areas(const std::vector<geometry_msgs::Pose> &found_objects) {
            std::vector<geometry_msgs::Pose> result;
            ignore_areas_mutex_.lock();
            std::vector<navigation_msgs::Area> areas(ignore_areas_);
            ignore_areas_mutex_.unlock();

            for (auto obj : found_objects){
                bool inside = false;
                for (auto area : areas){
                    if(get_dist_xy(obj.position, area.center) <= area.radius){
                        inside = true;
                        break;
                    }
                }
                if(!inside){
                    result.push_back(obj);
                }
            }

            return result;
        }

        /**
         * Calculates the new confidence based on the prior and observed confidence
         * @param prior
         * @param observation
         * @return new confidence
         */
        int calculate_confidence(const int prior, const int observation){
            double odds = exp(log(prior / (100.0 - prior)) + log(observation / (100.0 - observation)));
            return std::min(static_cast<int>(odds / (1.0 + odds) * 100.0) + 1, 99);
        }

        /**
         * Checks if a given object pose is inside the update bounds for the confidence
         * @param object_pose
         * @param robot_transform
         * @return true if inside
         */
        bool in_update_range(const geometry_msgs::Pose &object_pose, const tf::StampedTransform &robot_transform){
            geometry_msgs::PoseStamped ps, ps_out;
            ps.pose = object_pose;
            ps.header.frame_id = global_frame_;
            ps.header.stamp = robot_transform.stamp_;
            tf_listener_.transformPose(robot_base_frame_, ps, ps_out);
            double angle = atan2(ps_out.pose.position.y, ps_out.pose.position.x);
            double dist = sqrt(pow(ps_out.pose.position.x, 2) + pow(ps_out.pose.position.y, 2));
            return dist >= min_update_range_ && dist <= max_update_range_ && angle >= min_angle_ && angle <= max_angle_;
        }

        /**
         * Updates the confidence of all objects.
         * New objects are assigned a confidence of 50
         * Old objects outside the update-area remain untouched
         * Old objects inside the update-area get their confidence increased or decreased depending on whether their
         * pose is contained in the found_objects
         * @param found_objects
         */
        void update_confidence(const std::vector<geometry_msgs::Pose> &found_objects){
            tf::StampedTransform robot_transform;
            try{
                // Get robot transform
                tf_listener_.lookupTransform(robot_base_frame_, global_frame_, ros::Time(0), robot_transform);
                /*
                 * In order to identify new objects a seperate list with the indicies of the poses in found_objects is
                 * created.
                 * Should the pose of the found_object later match an existing object the index is removed from the list
                 * At the end the remaining objects that are in the update area are added as new objects.
                 */
                std::list<int> new_objects;
                for(int i=0; i<found_objects.size(); i++){
                    new_objects.push_back(i);
                }
                // Match the old object poses with the newly found objects
                int i_old_object = 0;
                for(auto object : objects_) {
                    bool iur = in_update_range(object, robot_transform);
                    bool found = false;
                    int i_new_object = 0;
                    for(auto found_object : found_objects){
                        double dist = get_dist_xy(found_object.position, object.position);
                        // old object found
                        if(dist <= max_dist_error_){
                            // old object found and in update-area
                            if(iur) {
                                confidence_[i_old_object] = calculate_confidence(confidence_[i_old_object], p_obj_);
                            }
                            // remove found_object from new objects
                            new_objects.remove(i_new_object);
                            found = true;
                        }
                        i_new_object++;
                    }
                    // Old object not found in update-area
                    if(!found && iur) {
                        confidence_[i_old_object] = calculate_confidence(confidence_[i_old_object], p_free_);
                    }
                    i_old_object++;
                }
                // add new objects
                for(auto new_object_i : new_objects){
                    objects_.push_back(found_objects[new_object_i]);
                    confidence_.push_back(50);
                }
            } catch (tf::TransformException ex){
                ROS_WARN_STREAM("Unable to update object confidence: " << ex.what());
            }
        }

        /**
         * Gets all objects from the costmap
         * @return
         */
        std::vector<geometry_msgs::Pose> get_objects() {
            std::vector<geometry_msgs::Pose> result;
            costmap_2d::Costmap2D cm(*costmap_->getCostmap());

            /*
             * saves the label of each field in the costmap
             * -1 free (disregarded)
             * 0+ part of said object
             */
            std::vector<int> label(cm.getSizeInCellsX()*cm.getSizeInCellsY());
            std::vector<std::list<pos>> objects;

            for (unsigned int x = 0; x < cm.getSizeInCellsX(); x++) {
                for (unsigned int y = 0; y < cm.getSizeInCellsY(); y++) {
                    // Only process occupied cells
                    if (cm.getCost(x, y) >= threshold_occupied_) {
                        unsigned int i = cm.getIndex(x,y);
                        pos p;
                        p.i = i;
                        p.x = x;
                        p.y = y;
                        int left = 0 < x ? label[i -1] : -1; // label of the left neighbour
                        int top = 0 < y ? label[cm.getIndex(x, y-1)] : -1; // label of top neighbour
                        //new object (left and top are not part of an object)
                        if (-1 == left && -1 == top){
                            objects.push_back(std::list<pos>());
                            objects[objects.size()-1].push_back(p);
                            label[p.i] = objects.size()-1;
                        //part of top object (left not part of an object)
                        } else if (-1 == left) {
                            objects[top].push_back(p);
                            label[i] = top;
                        //part of top and left object (top and left are part of the same object or top is not an object)
                        } else if (-1 == top || top == left) {
                            objects[left].push_back(p);
                            label[i] = left;
                        //connects top and left object (left and top are part of different objects)
                        } else {
                            //get the higher and lower label
                            //the object of the higher label is merged into the object of the lower label
                            int l_l = top < left ? top : left;
                            int h_l = top < left ? left : top;
                            for(pos i_p : objects[h_l]){
                                label[i_p.i] = l_l;
                            }
                            objects[l_l].splice(objects[l_l].end(), objects[h_l]);
                        }
                    } else {
                        label[cm.getIndex(x,y)] = -1;
                    }
                }
            }
            for (auto object : objects){
                // filter out things that are to big or small
                // calculate pose of object as average
                if (min_object_size_ <= object.size() && object.size() <= max_object_size_){
                    geometry_msgs::Pose pose;
                    pose.orientation.w = 1.0;
                    for(pos p : object){
                        double wx, wy;
                        cm.mapToWorld(p.x, p.y, wx, wy);
                        pose.position.x += wx;
                        pose.position.y += wy;
                    }
                    pose.position.x /= object.size();
                    pose.position.y /= object.size();
                    result.push_back(pose);
                }
            }
            return result;
        }

        /**
         * Publishes the poses of the found objects
         * @param poses
         */
        void publish_objects(std::vector<geometry_msgs::Pose> poses) {
            geometry_msgs::PoseArray pa;
            pa.header.frame_id = global_frame_;
            pa.header.stamp = ros::Time::now();
            pa.poses = poses;
            object_publisher_.publish(pa);
        }
    };

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_finder");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    object_finder::ObjectFinder object_finder(buffer);
    object_finder.run();

    ros::spin();
    return 0;
}