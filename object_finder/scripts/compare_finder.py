import rospy
import rosbag
from geometry_msgs.msg import PoseArray, Pose
from matplotlib import pyplot as ppl
import math

max_dist_detection = 0.06


def get_distance(pose, xy):
    return math.sqrt(math.pow(abs(pose.position.x - xy[0]), 2) + math.pow(abs(pose.position.y - xy[1]), 2))


def get_closest_object(pose, objects):
    dist = 9999.99
    index = -1
    for i, obj in enumerate(objects):
        if get_distance(pose, obj) < dist:
            index = i
            dist = get_distance(pose, obj)
    return index, dist


def evaluate(bagfile, object_finder, objects):
    true_positive = 0
    false_positive = 0
    false_negative = 0
    total_objects = 0
    objects_found = [0] * len(objects)
    pose_error = 0.0

    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages(topics=[object_finder]):
        found_objects = []
        for pose in msg.poses:
            i, d = get_closest_object(pose, objects)
            if d <= max_dist_detection:
                if i in found_objects:
                    false_positive += 1
                else:
                    true_positive += 1
                    found_objects.append(i)
                    objects_found[i] += 1
                    pose_error += d
            else:
                false_positive += 1
        false_negative += len(objects) - len(found_objects)
        total_objects += len(objects)

    precision = float(true_positive) / (float(true_positive + false_positive))
    recall = float(true_positive) / (float(true_positive + false_negative))
    f1_score = 2*(recall * precision) / (recall + precision)
    pose_error = pose_error / true_positive

    print("======================================================")
    print(object_finder)
    print("true_positive: {}".format(true_positive))
    print("false_positive: {}".format(false_positive))
    print("false_negative: {}".format(false_negative))
    print("")
    print("precision: {}".format(precision))
    print("recall: {}".format(recall))
    print("f1_score: {}".format(f1_score))
    print("")
    print("pose_error: {}".format(pose_error))
    print("objects_found: {}".format(objects_found))
    print("======================================================")



if __name__ == '__main__':
    _bagfile = "/home/marc/of_test_bag/2021-04-14-21-09-47.bag"
    _objects = [(-2.01, 1.00), (-1.03, 4.93), (1.98, 3.01), (1.84, -0.52), (3.99, 0.99), (2.10, 2.85)]
    _object_finder = ["/object_finder_default/object_finder",
                     "/object_finder_headcam_only/object_finder",
                     "/object_finder_inflation_10/object_finder",
                     "/object_finder_inflation_20/object_finder",
                     "/object_finder_laser_marking_only/object_finder",
                     "/object_finder_laser_marking_only_inflation/object_finder",
                     "/object_finder_laser_only/object_finder"]
    for of in _object_finder:
        evaluate(_bagfile, of, _objects)