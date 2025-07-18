//MANI RADHAKRISHNAN
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cfloat>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#define MY_DICTIONARY	(cv::aruco::DICT_6X6_250)
#define MARKER_ID		23
#define MARKER_SIZE		800
#define MARKER_BORDER	1

bool sort3DPointByDepth(const Eigen::Array4f a, const Eigen::Array4f b)
{
	return a[3] < b[3];
}

bool find_centre(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f* r)
{
	cv::Point2f x = o2 - o1;
	cv::Point2f d1 = p1 - o1;
	cv::Point2f d2 = p2 - o2;
	float cross = d1.x*d2.y -d1.y*d2.x;
	if(abs(cross) < /*EPS*/1e-8)
		return false;
	double t1 = (x.x*d2.y - x.y*d2.x)/cross;
	*r = o1 + d1*t1;
	return true;
}

void record_debug_data(void)
{
        char datecode[80];
        std::stringstream copy_command;

        std::time_t t = std::time(NULL);
        std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));

        copy_command.str("");
        copy_command << "cp " YANTHRA_DATA_OUTPUT_DIR "/output_image.jpg " << YANTHRA_DATA_DEBUG_DIR "/" << datecode << "_output_image.jpg";
        system(copy_command.str().c_str());
}

void getStatisticalDepth(const pcl::PointCloud<pcl::PointXYZ> &cloud,
									const std::vector<int> &indices,
									pcl::PointXYZ &depth)
{
	Eigen::Array4f min_p, max_p;
	std::vector<Eigen::Array4f> depth_array;
	min_p.setConstant(FLT_MAX);
	max_p.setConstant(-FLT_MAX);
	size_t number_of_points = 0;

	// If the data is dense, we don't need to check for NaN
	if(cloud.is_dense)
	{
		std::cout << "error this case is not handled" << std::endl;
		for(size_t i = 0; i < cloud.points.size(); ++i)
		{
			pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
			min_p = min_p.min(pt);
			max_p = max_p.max(pt);
		}
	}
	// NaN or Inf values could exist => check for them
	else
	{
		for(size_t i = 0; i < indices.size(); ++i)
		{
			// Check if the point is invalid
			if (!pcl_isfinite(cloud.points[indices[i]].x) || 
				!pcl_isfinite(cloud.points[indices[i]].y) || 
				!pcl_isfinite(cloud.points[indices[i]].z))
			{
				continue;
			}
			pcl::Array4fMapConst pt = cloud.points[indices[i]].getArray4fMap();
			depth_array.push_back(pt);
			min_p = min_p.min(pt);
			max_p = max_p.max (pt);
			number_of_points++;
		}
	}
	if(number_of_points == 1)
	{ 
		depth.x = depth_array[0][0];
		depth.y = depth_array[0][1];
		depth.z = depth_array[0][2];
	}
	else if(number_of_points > 1) 
	{ 
		sort(depth_array.begin(), depth_array.end(), sort3DPointByDepth);
		depth.x = depth_array[number_of_points/2][0];
		depth.y = depth_array[number_of_points/2][1];
		depth.z = depth_array[number_of_points/2][2];
	}
	else
	{
		std::cout << "Returning nan depth" << std::endl;
		const float bad_point = std::numeric_limits<float>::quiet_NaN();
		depth.x = depth.y = depth.z = bad_point;
	}
}

/* 
	Detect the Marker Postion within the Image
*/
class aruco_detect
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
public:
	cv::Mat input_img;
	std::vector<pcl::PointXYZ> detected_points;
private:
	bool capture_image(void)
	{
		pcl::PCDReader reader;
		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                remove(YANTHRA_DATA_INPUT_DIR "/img100.jpg");
                remove(YANTHRA_DATA_INPUT_DIR "/points100.pcd");
		system(RS_CAPTURE_S640_PROGRAM);
		input_img = cv::imread(YANTHRA_DATA_INPUT_DIR "/img100.jpg", cv::IMREAD_COLOR);
		// check whether the file is read 
		if (access( YANTHRA_DATA_INPUT_DIR "/img100.jpg",F_OK ) !=0){  //return (false) ; 
		std::cout << "no image" << std::endl ;	
	        return false;	
		}
		reader.read<pcl::PointXYZ>(YANTHRA_DATA_INPUT_DIR "/points100.pcd", *point_cloud);
		input_cloud = point_cloud;
		return (true );
	}
	pcl::PointXYZ convert_2d_to_3d(cv::Point2f point_2d)
	{
		pcl::PointXYZ point_3d;
		std::vector<int> pt_indices;
		int i, j, x, y, dx, dy;
		int direction_array[8][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1} };
		
		for(i = 0; i < 8; i++)
		{
			dx = direction_array[i][0];
			dy = direction_array[i][1];
			for(j = 1; j <= 5; j++)
			{
				x = point_2d.x + (j * dx);
				y = point_2d.y + (j * dy);
				
				if((x >= input_cloud->width) || (x < 0))
					continue;
				if((y >= input_cloud->height) || (y < 0))
					continue;
				pt_indices.push_back((input_cloud->width * y) + x);
			}
		}
		getStatisticalDepth(*input_cloud, pt_indices, point_3d);
		
		return point_3d;
	}
public:
	bool aruco_detect_marker(void)
	{
		bool marker_detected = false;
		cv::Scalar line_color;
		int index = 0;
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f> > markerCorners;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(MY_DICTIONARY);
		
		/* Capture the image */
		std::cout << "aruco_detect_marker : Calling capture_image" << std::endl ;	
		if (!capture_image() ) return (false)  ;
		std::cout << "aruco_detect_marker : Completed capture_image" << std::endl ;
		cv::aruco::detectMarkers(input_img, dictionary, markerCorners, markerIds);
		for(index = 0; index < markerIds.size(); index++)
		{
			if(markerIds[index] == MARKER_ID)
			{
				/* If desired marker is deteced, convert the corner points from 2d to 3d, and save in output array i.e. detected_points */
				detected_points.clear();
				detected_points.push_back(convert_2d_to_3d(markerCorners[index][0]));
				detected_points.push_back(convert_2d_to_3d(markerCorners[index][1]));
				detected_points.push_back(convert_2d_to_3d(markerCorners[index][2]));
				detected_points.push_back(convert_2d_to_3d(markerCorners[index][3]));
				
				/* Check the valadity of detected corner points */
				if(std::isnan(detected_points[0].z) ||
				   std::isnan(detected_points[1].z) ||
				   std::isnan(detected_points[2].z) ||
				   std::isnan(detected_points[3].z))
				{
					marker_detected = false;
					line_color = cv::Scalar(0, 0, 255);
				}
				else
				{
					marker_detected = true;
					line_color = cv::Scalar(0, 255, 0);
				}
				/* Draw rectangle with detected corner points */
				cv::circle(input_img, markerCorners[index][0], 5, line_color, 1, 8);
				cv::line(input_img, markerCorners[index][0], markerCorners[index][1], line_color);
				cv::line(input_img, markerCorners[index][1], markerCorners[index][2], line_color);
				cv::line(input_img, markerCorners[index][2], markerCorners[index][3], line_color);
				cv::line(input_img, markerCorners[index][3], markerCorners[index][0], line_color);
			}
		}
		return marker_detected;
	}
	
public:
	aruco_detect(void) {}
	~aruco_detect(void) {}
};

/*
	Generate Marker Image, to be used for calibration
*/
class aruco_generate
{
public:
	cv::Mat generate(void)
	{
		cv::Mat marker;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(MY_DICTIONARY);
		cv::aruco::drawMarker(dictionary, MARKER_ID, MARKER_SIZE, marker, MARKER_BORDER);
		
		return marker;
	}
	aruco_generate(void) {}
	~aruco_generate(void) {}
};

aruco_detect detectMarker;

int main(int argc, char** argv)
{

	std::cout << " aruco_finder :: Deleting old centroid.txt file if it exists" << std::endl ;
	system(" echo /bin/rm  /home/ubuntu/.ros/centroid.txt /home/ubuntu/input/points100.pcd /home/ubuntu/input/img100.jpg") ;	
	if (system("/bin/rm  /home/ubuntu/.ros/centroid.txt")) {
			std::cerr << " aruco_finder :: Not able to delete /home/ubuntu/.ros/centroid.txt /home/ubuntu/input/points100.pcd /home/ubuntu/input/img100.jpg " << std::endl ;
	} else {
	    system ("ls -l /home/ubuntu/.ros ; date ") ;
	} 
	  
	//TODO : Change to CMakelist with related to a run_directory
	std::ofstream centroid_file("/home/ubuntu/.ros/centroid.txt");
	//std::ofstream centroid_file(YANTHRA_DATA_OUTPUT_DIR + "centroid.txt");
	std::cout << " aruco_detect:: main >> Calling Marker detection " << std::endl;
	if (detectMarker.aruco_detect_marker() == true) {
		centroid_file << detectMarker.detected_points[0].x << " " << detectMarker.detected_points[0].y << " " << detectMarker.detected_points[0].z << std::endl;
		centroid_file << detectMarker.detected_points[1].x << " " << detectMarker.detected_points[1].y << " " << detectMarker.detected_points[1].z << std::endl;
		centroid_file << detectMarker.detected_points[2].x << " " << detectMarker.detected_points[2].y << " " << detectMarker.detected_points[2].z << std::endl;
		centroid_file << detectMarker.detected_points[3].x << " " << detectMarker.detected_points[3].y << " " << detectMarker.detected_points[3].z << std::endl;
	} else {
		centroid_file << 0 << 0 << 0 << std::endl;
		std::cout << "Marker detection failed" << std::endl;
		return 1 ;
	}

	cv::imwrite(YANTHRA_DATA_OUTPUT_DIR "/output_image.jpg", detectMarker.input_img);
  record_debug_data();
	
	return 0;
}
