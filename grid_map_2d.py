import numpy as np
from nav_msgs.msg import OccupancyGrid,Odometry
import rospy
from visualization_msgs.msg import Marker,MarkerArray
from shapely import Polygon,Point
from utils import is_point_inside_polygon,publish_obs_marker
from scipy import ndimage
'''
coor is in the world coor
mapcoor is in the gird map
'''


class Nav_map():
    def __init__(self):
        self.map_subscriber = rospy.Subscriber("/map",OccupancyGrid,self.MapCallback)
        # self.marker_pub = rospy.Publisher('/obstacle_marker', MarkerArray, queue_size=10)
        self.marker_pub = rospy.Publisher('obs_markers_array', MarkerArray, queue_size=10)
        self.map = None
        self.map_flag = False
        self.resolution = None #0.05
        self.plan_map = None
        self.obs_points = set()
        self.dis_trans = None
        # origin_x,y = -50,-50

    def MapCallback(self,msg):
        self.map = msg
        self.map_flag = True
        self.resolution = msg.info.resolution

    def construct_planmap(self):
        obs_grid_coor = self.get_obs_in_map()

    def get_distance_transform(self):
        while True:
            if self.map_flag:
                A = np.ones((self.map.info.height,self.map.info.width))
                for j in range(self.map.info.height):
                    for i in range(self.map.info.width):
                        if self.map.data[j*self.map.info.width+i] >=80 :
                            A[j][i] = 0
                dis = ndimage.distance_transform_edt(A)
                
                self.dis_trans = dis
                import matplotlib.pyplot as plt

                # plt.imshow(dis, cmap='hot', interpolation='nearest')
                # plt.colorbar()
                # plt.show()
                return dis

    def get_obs_in_map(self):
        #return the coordinate of the obstacle in the 2D occupancy map  not map coordinate
        obs_points = []
        if self.map_flag:
            for i in range (0,len(self.map.data)):
                if self.map.data[i] == 100:
                    coor = self.index_to_mapcoor(i)
                    obs_points.append(coor)
            for point in obs_points:
                self.obs_points.add(tuple(point))
            return self.obs_points
        else:
            rospy.logwarn("Map data not available yet.  err from get_obs_in_map()")
            return obs_points

    def publish_obs_marker(self,point):
                #return the coordinate of the obstacle in the 2D occupancy map  not map coordinate
        obs_points = []
        if self.map_flag:
            for i in range (0,len(self.map.data)):
                if self.map.data[i] == 100:
                    coor = self.index_to_coor(i)
                    obs_points.append(coor)
            print("len of obs_points",len(obs_points))
            #把obs_points 全部发布出去
            marker_array = MarkerArray()
            for i, point in enumerate(obs_points):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0
                marker.id = i
                marker_array.markers.append(marker)

            self.marker_pub.publish(marker_array)
            print("successfully pub! obspoints")
            # print("obs_points",obs_points)
            for point in obs_points:
                self.obs_points.add(tuple(point))
            return self.obs_points
        else:
            rospy.logwarn("Map data not available yet.  err from get_obs_in_map()")
            return obs_points

    def index_to_coor(self,index):
        if not self.map_flag:
            rospy.logwarn("Map data not available yet.")
            return None

        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        map_resolution = self.map.info.resolution

        map_x = index % self.map.info.width
        map_y = index // self.map.info.width

        map_x = map_x * map_resolution + map_origin_x
        map_y = map_y * map_resolution + map_origin_y

        return np.array([map_x, map_y])
    
    def index_to_mapcoor(self,index):
        if not self.map_flag:
            rospy.logwarn("Map data not available yet.")
            return None

        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        map_resolution = self.map.info.resolution

        map_x = index % self.map.info.width
        map_y = index // self.map.info.width
        return np.array([map_x, map_y])

    def coor_to_index(self,point):
        if not self.map_flag:
            rospy.logwarn("Map data not available yet.")
            return None

        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        map_resolution = self.map.info.resolution

        map_x = point[0]
        map_y = point[1]

        if map_x < 0 or map_x >= self.map.info.width or map_y < 0 or map_y >= self.map.info.height:
            rospy.logwarn("Point is outside of the map boundaries.")
            return None
        # print("self.map.info.width ",self.map.info.width)
        # print("map_y",map_y)
        # print("map_x",map_x)
        map_index = map_y * self.map.info.width + map_x
        return map_index
    
    def mapcoor_tocoor(self,point):#correct
        if not self.map_flag:
            rospy.logwarn("Map data not available yet.")
            return None

        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        map_resolution = self.map.info.resolution

        map_x = point[0]*map_resolution + map_origin_x
        map_y = point[1]*map_resolution + map_origin_y
        return np.array([map_x, map_y])
    
    def coor_to_mapcoor(self,point):
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        map_resolution = self.map.info.resolution
        map_x = int((point[0] - map_origin_x) // map_resolution)
        map_y = int((point[1] - map_origin_y) // map_resolution)

        return [map_x,map_y]
    #TODO
    def find_waypoint_around_obs(self,waypoint,obs_point,searh_distance,robot):# arg is coordinate not occupancy
        obs_inflate_x_min = obs_point[0] - searh_distance
        obs_inflate_x_max = obs_point[0] + searh_distance
        obs_inflate_y_min = obs_point[1] - searh_distance
        obs_inflate_y_max = obs_point[1] + searh_distance
        #把障碍物看成是一个方形的区域找到他的边界
        obs_polygon = Polygon([(obs_inflate_x_min,obs_inflate_y_min),(obs_inflate_x_max,obs_inflate_y_min),(obs_inflate_x_max,obs_inflate_y_max),(obs_inflate_x_min,obs_inflate_y_max)])
        # publish_obs_marker([obs_inflate_x_min,obs_inflate_y_min])
        # publish_obs_marker([obs_inflate_x_max,obs_inflate_y_min])
        # publish_obs_marker([obs_inflate_x_min,obs_inflate_y_max])
        # publish_obs_marker([obs_inflate_x_max,obs_inflate_y_max])
        old_flag = False
        start_index = 0
        end_index = 0
        if is_point_inside_polygon(robot.pose[:2],obs_polygon):
            print("robot is inside the obs.")
            for i in range(len(waypoint)):
                new_flag = is_point_inside_polygon(waypoint[i],obs_polygon)
                if new_flag == True and old_flag == False:
                    start_index = i+1
                    
                if new_flag == False and old_flag == True:
                    end_index = i-1
                    start_index = 0
                    break
                old_flag = new_flag
                
            return np.array([waypoint[start_index],waypoint[end_index]]), start_index, end_index

        for i in range(len(waypoint)):
            new_flag = is_point_inside_polygon(waypoint[i],obs_polygon)
            if new_flag == True and old_flag == False:
                start_index = i+1
            if new_flag == False and old_flag == True:
                end_index = i-1

                break
            old_flag = new_flag
        publish_obs_marker(waypoint[start_index],"start_marker")
        publish_obs_marker(waypoint[end_index],"end_marker")
        # publish_obs_marker(waypoint[end_index])
        return np.array([waypoint[start_index],waypoint[end_index]]), start_index, end_index
        


# #method 2 directly use line and intersect to find the intersection point and add these points to waypoint
#     from shapely.geometry import LineString

#     def find_intersection_points(self, waypoint, obs_polygon):
#         intersection_points = []
        
#         for i in range(len(waypoint) - 1):
#             line = LineString([waypoint[i], waypoint[i+1]])
#             intersections = line.intersection(obs_polygon)
            
#             if intersections.is_empty:
#                 continue
            
#             if intersections.geom_type == 'Point':
#                 intersection_points.append((intersections.x, intersections.y))
#             elif intersections.geom_type == 'MultiPoint':
#                 for point in intersections:
#                     intersection_points.append((point.x, point.y))
        
#         return intersection_points

if __name__ == "__main__":
    rospy.init_node("nav_map")
    nav_map = Nav_map()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if nav_map.map_flag:
            print("map is available")
            # nav_map.get_obs_in_map()
            nav_map.get_distance_transform()
        rate.sleep()
    rospy.spin()