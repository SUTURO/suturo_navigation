#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <object_finder/ObjectFinderConfig.h>

struct pos {
    unsigned int x;
    unsigned int y;
    unsigned int i;
};

namespace object_finder {

    class ObjectFinder {
    public:
        ObjectFinder(tf2_ros::Buffer &tf) {
            ros::NodeHandle nh("~");
            costmap_ = std::make_shared<costmap_2d::Costmap2DROS>("costmap", tf);
            object_publisher_ = nh.advertise<geometry_msgs::PoseArray>("object_finder", 10);
            ros::param::param<std::string>("global_frame_", global_frame_, "map");
            ros::param::param<int>("frequency_", frequency_, 1);

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

        void reconfigure_callback(ObjectFinderConfig &config, uint32_t level) {
            ROS_INFO("Reconfigure: min_object_size_: %d max_object_size_: %d threshold_occupied_: %d", config.min_object_size, config.max_object_size, config.threshold_occupied);
            min_object_size_ = config.min_object_size;
            max_object_size_ = config.max_object_size;
            threshold_occupied_ = config.threshold_occupied;
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
                    cm.mapToWorld(object.front().x, object.front().y, pose.position.x, pose.position.y);
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

        void run() {
            ros::Rate loop_rate(frequency_);
            while (ros::ok()) {
                publish_objects(get_objects());
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
        ros::Publisher object_publisher_;
        std::string global_frame_;
        int max_object_size_, min_object_size_, threshold_occupied_, frequency_;
        dynamic_reconfigure::Server<ObjectFinderConfig> *dsrv_;
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