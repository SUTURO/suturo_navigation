#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <object_finder/ObjectFinderConfig.h>
#include <tf/transform_listener.h>

#include <cmath>

struct pos {
    unsigned int x;
    unsigned int y;
    unsigned int i;
};

static double get_dist_xy(const geometry_msgs::Point &a, const tf::Vector3 &b)
{
    return sqrt(pow(abs(a.x - b.getX()), 2) + pow(abs(a.y - b.getY()), 2));
}

static double get_angle_xy(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    return atan2(a.y - b.y, a.x - b.x);
}

namespace object_finder {

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
            ros::param::param<double>("update_range", update_range_, 3.0);
            ros::param::param<double>("max_dist_error", max_dist_error_, 0.05);
            dsrv_ = new dynamic_reconfigure::Server<ObjectFinderConfig>(nh);
            dynamic_reconfigure::Server<ObjectFinderConfig>::CallbackType cb = boost::bind(
                    &ObjectFinder::reconfigure_callback, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        ~ObjectFinder()
        {
            if (dsrv_)
                delete dsrv_;
        }

        void run() {
            objects_.clear();
            confidence_.clear();
            ros::Rate loop_rate(frequency_);
            while (ros::ok()) {
                std::vector<geometry_msgs::Pose> found_objects = get_objects();
                if(use_probability_){
                    std::vector<geometry_msgs::Pose> p_objects;
                    update_confidence(found_objects);
                    int i = 0;
                    for(auto conf : confidence_){
                        if(conf >= min_confidence_){
                            p_objects.push_back(objects_[i]);
                        }
                        i++;
                    }
                    publish_objects(p_objects);
                } else{
                    publish_objects(found_objects);
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
        std::vector<geometry_msgs::Pose> objects_;
        std::vector<int> confidence_;
        ros::Publisher object_publisher_;
        int max_object_size_, min_object_size_, threshold_occupied_, frequency_, min_confidence_, p_obj_, p_free_;
        dynamic_reconfigure::Server<ObjectFinderConfig> *dsrv_;
        bool use_probability_;
        double min_angle_, max_angle_, update_range_, max_dist_error_;
        tf::TransformListener tf_listener_;
        std::string robot_base_frame_, global_frame_;

        void reconfigure_callback(ObjectFinderConfig &config, uint32_t level) {
            ROS_INFO_STREAM("Reconfiguring");
            min_object_size_ = config.min_object_size;
            max_object_size_ = config.max_object_size;
            threshold_occupied_ = config.threshold_occupied;
            min_confidence_ = config.threshold_occupied;
            p_obj_ = config.p_obj;
            p_free_ = config.p_free;
            min_angle_ = config.min_angle;
            max_angle_ = config.max_angle;
            update_range_ = config.update_range;
            max_dist_error_ = config.max_dist_error;
            use_probability_ = config.use_probability;
        }

        int calculate_confidence(const int prior, const int observation){
            double odds = exp(log(prior / (100.0 - prior)) + log(observation / (100.0 - observation)));
            return std::min(static_cast<int>(1.0 - (1.0 / (1.0 + odds)) * 100.0) + 1, 99);
        }

        bool in_update_range(const geometry_msgs::Pose &object_pose, const tf::StampedTransform &robot_transform){
            geometry_msgs::PoseStamped ps, ps_out;
            ps.pose = object_pose;
            ps.header.frame_id = global_frame_;
            ps.header.stamp = robot_transform.stamp_;
            tf_listener_.transformPose(robot_base_frame_, ps, ps_out);
            double angle = tf::getYaw(ps_out.pose.orientation);
            double dist = sqrt(pow(ps_out.pose.position.x, 2) + pow(ps_out.pose.position.y, 2));
            return dist <= update_range_ && angle >= min_angle_ && angle <= max_angle_;
        }

        void update_confidence(const std::vector<geometry_msgs::Pose> &found_objects){
            tf::StampedTransform robot_transform;
            try{
                tf_listener_.lookupTransform(robot_base_frame_, global_frame_, ros::Time(0), robot_transform);
                std::list<int> new_objects;
                for(int i=0; i<found_objects.size(); i++){
                    new_objects.push_back(i);
                }
                int i = 0;
                for(auto object : objects_) {
                    if(in_update_range(object, robot_transform)){
                        bool found = false;
                        for(auto found_object : found_objects){
                            if(get_dist_xy(found_object.position, robot_transform.getOrigin()) <= max_dist_error_){
                                confidence_[i] = calculate_confidence(confidence_[i], p_obj_);
                                new_objects.remove(i);
                                found = true;
                            }
                        }
                        if(!found) {
                            confidence_[i] = calculate_confidence(confidence_[i], p_free_);
                        }
                    }
                    i++;
                }
                for(auto new_object_i : new_objects){
                    objects_.push_back(found_objects[new_object_i]);
                    confidence_.push_back(50);
                }
            } catch (tf::TransformException ex){
                ROS_WARN_STREAM("Unable to update object confidence: " << ex.what());
            }
        }

        std::vector<geometry_msgs::Pose> get_objects() {
            std::vector<geometry_msgs::Pose> result;
            costmap_2d::Costmap2D cm(*costmap_->getCostmap());

            std::vector<int> label(cm.getSizeInCellsX()*cm.getSizeInCellsY());
            std::vector<std::list<pos>> objects;

            for (unsigned int x = 0; x < cm.getSizeInCellsX(); x++) {
                for (unsigned int y = 0; y < cm.getSizeInCellsY(); y++) {
                    //TODO: Add threashold for occupied to reconfgurable parameters
                    //Only process occupied cells
                    if (cm.getCost(x, y) >= threshold_occupied_) {
                        unsigned int i = cm.getIndex(x,y);
                        pos p;
                        p.i = i;
                        p.x = x;
                        p.y = y;
                        int left = 0 < x ? label[i -1] : -1;
                        int top = 0 < y ? label[cm.getIndex(x, y-1)] : -1;
                        //new object
                        if (-1 == left && -1 == top){
                            objects.push_back(std::list<pos>());
                            objects[objects.size()-1].push_back(p);
                            label[p.i] = objects.size()-1;
                        } else if (-1 == left) {
                            objects[top].push_back(p);
                            label[i] = top;
                        } else if (-1 == top || top == left) {
                            objects[left].push_back(p);
                            label[i] = left;
                        } else {
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
                //TODO: add param for this
                //Filter out things that are to big for the HSR
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