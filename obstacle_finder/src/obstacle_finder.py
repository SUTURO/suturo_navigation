#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException
import math
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseArray, Pose, PoseStamped


class Cluster:
    def __init__(self):
        self.num_points = 0
        self.points = []


class ObstacleFinder:

    def __init__(self):
        self._map = None
        self._pose = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)

        self._map_sub = rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        self._laser_sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, self.find_objects)
        #self._pose_sub = rospy.Subscriber("/global_pose", PoseStamped, self.update_pose)
        self._pub = rospy.Publisher("object_finder", PoseArray, queue_size=10)

        self._occ_threshold = 40
        self._min_scans_cluster = 10
        self._min_percentage_covered = 0.7
        self._rad_neighbour = 0.1
        self._error_map = 0.1
        self._use_every_n_scan = 15
        self._scan = 0

        self._clusters = []

    def update_pose(self, pose):
        self._pose = pose.pose

    def update_map(self, map):
        self._map = map

    def point_equal_neighbour(self, cluster, p):
        for p_i in cluster.points:
            if self.equal_or_neighbour(p_i, p):
                return True
        return False

    def equal_or_neighbour(self, p1, p2):
        delta = int(self._rad_neighbour / self._map.info.resolution)
        for i in range(-delta, delta, 1):
            for j in range(-delta, delta, 1):
                if p1.x == p2.x + i and p1.y == p2.y + j:
                    return True
        return False

    def cluster_in_map(self, cluster):
        matches = 0.0
        for p in cluster.points:
            delta = int(self._error_map / self._map.info.resolution)
            matched = False
            for i in range(-delta, delta, 1):
                for j in range(-delta, delta, 1):
                    if not matched and 0 <= p.y + j <= self._map.info.height - 1 and 0 <= p.x + i <= self._map.info.width - 1:
                        i_map = (p.y + j) * self._map.info.width + p.x + i
                        #i_map = (p.x + i) * self._map.info.height + p.y +j
                        if self._map.data[i_map] >= self._occ_threshold:
                            matches += 1
                            matched = True
        perc = matches / cluster.num_points
        print(perc)
        if perc >= self._min_percentage_covered:
            return True
        return False

    def pub_cluster(self, clusters, stamp):
        # print(clusters.__sizeof__())
        markers = []
        i = 0
        for c in clusters:
            # print(c)
            '''
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.CUBE
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.orientation.w = 0.1
            p2 = Point()
            for p in c.points:
                p2.x += p.x * self._map.info.resolution
                p2.y += p.y * self._map.info.resolution
            p2.x /= c.num_points
            p2.y /= c.num_points
            p2.x += self._map.info.origin.position.x
            p2.y += self._map.info.origin.position.y
            # print(p2)
            marker.pose.position = p2
            markers.append(marker)
            i += 1
            '''

            p2 = Point()
            for p in c.points:
                p2.x += p.x * self._map.info.resolution
                p2.y += p.y * self._map.info.resolution
            p2.x /= c.num_points
            p2.y /= c.num_points
            p2.x += self._map.info.origin.position.x
            p2.y += self._map.info.origin.position.y
            p2.z = c.points[0].z

            pose = Pose()
            pose.position = p2
            pose.orientation.w = 1
            markers.append(pose)
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = stamp
        pa.poses = markers
        self._pub.publish(pa)

    def find_objects(self, laser_scan):
        self._scan += 1
        if not self._map == None and self._scan == self._use_every_n_scan:
            self._scan = 0
            first = True
            clusters = []
            cur_Cluster = Cluster()

            angle = laser_scan.angle_min
            try:
                map_laser = self._tf_buffer.lookup_transform(self._map.header.frame_id, laser_scan.header.frame_id, rospy.Time(0))
            except (LookupException, ConnectivityException):
                print("no tf scan aborted")
                self._scan = self._use_every_n_scan - 1
                return
            #pose = self._pose

            theta = euler_from_quaternion(
                [map_laser.transform.rotation.x, map_laser.transform.rotation.y, map_laser.transform.rotation.z,
                 map_laser.transform.rotation.w])[2]

            #theta = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

            # print(map_laser, theta)
            for i, range in enumerate(laser_scan.ranges):
                if laser_scan.range_min <= range <= laser_scan.range_max:
                    # og = OccupancyGrid()
                    x = range * math.cos(angle + theta) + map_laser.transform.translation.x
                    y = range * math.sin(angle + theta) + map_laser.transform.translation.y
                    #x = range * math.cos(angle + theta) + pose.position.x
                    #y = range * math.sin(angle + theta) + pose.position.y
                    ix = int((x - self._map.info.origin.position.x) / self._map.info.resolution)
                    iy = int((y - self._map.info.origin.position.y) / self._map.info.resolution)
                    p = Point(ix, iy, map_laser.transform.translation.z)
                    # print("Obstacle at: ", x,y)
                    # marker.points.append(Point(x, y, 0))

                    # ToDo: Cluster scans onto one point. Combine points and check if there is
                    # an obstacle on the map at this position.
                    if not first and not self.point_equal_neighbour(cur_Cluster, p):
                        if self._min_scans_cluster <= cur_Cluster.num_points and not self.cluster_in_map(cur_Cluster):
                            clusters.append(cur_Cluster)
                        cur_Cluster = Cluster()
                    cur_Cluster.points.append(p)
                    cur_Cluster.num_points += 1
                    first = False
                angle += laser_scan.angle_increment
            if self._min_scans_cluster <= cur_Cluster.num_points and not self.cluster_in_map(cur_Cluster):
                clusters.append(cur_Cluster)
            self.pub_cluster(clusters, laser_scan.header.stamp)

            # self._pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node("obstacle_finder")
    of = ObstacleFinder()
    rospy.spin()
