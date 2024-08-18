#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP

#include "external/delaunator.hpp"
#include "Cone.hpp"
#include "Track.hpp"

/**
 * @brief Computes the Euclidean distance between two points.
 * 
 * This function calculates the Euclidean distance between two points in a 2D space. 
 * It uses the standard distance formula to compute the distance between the points 
 * represented by the `Point` objects.
 *
 * @author Anton Haes
 * 
 * @tparam T The type of the coordinates (e.g., `int`, `float`, `double`). 
 *           This should be a numeric type that supports arithmetic operations and 
 *           the `std::sqrt` function.
 * 
 * @param p1 The first point.
 * @param p2 The second point.
 * 
 * @return The Euclidean distance between the points `p1` and `p2`. The return type 
 *         is the same as the coordinate type `T`.
 */
template<typename T>
T distance(const Point<T>& p1, const Point<T>& p2) {
	T dx = p2.getX() - p1.getX();
	T dy = p2.getY() - p1.getY();
	return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Compares the distances of two points from a reference point.
 * 
 * This function calculates the Euclidean distances from a reference point to two other 
 * points, and then compares these distances. It returns `true` if the distance to the 
 * first point is smaller than the distance to the second point, and `false` otherwise.
 * 
 * @author Anton Haes
 *
 * @tparam T The type of the coordinates (e.g., `int`, `float`, `double`). 
 *           This should be a numeric type that supports arithmetic operations and 
 *           the `std::sqrt` function.
 * 
 * @param p1 The first point to compare.
 * @param p2 The second point to compare.
 * @param reference_point The reference point from which distances to `p1` and `p2` are measured.
 * 
 * @return `true` if the distance from `previous_point` to `p1` is less than the distance 
 *         from `previous_point` to `p2`; `false` otherwise.
 */
template<typename T>
bool compareDistance(const Point<T>& p1, const Point<T>& p2, const Point<T>& reference_point) {
	T dist1 = distance(reference_point, p1);
	T dist2 = distance(reference_point, p2);
	return dist1 < dist2;
}

template<typename T1, typename T2>
bool intersects(Point<T1>& p1, Point<T1>& p2, Point<T2>& p3, Point<T2>& p4) {
	// Calculate the orientation of triplet (p1, p2, p3)
	auto orientation1 = (p2.getY() - p1.getY()) * (p3.getX() - p2.getX()) - 
	                    (p3.getY() - p2.getY()) * (p2.getX() - p1.getX());

	// Calculate the orientation of triplet (p1, p2, p4)
	auto orientation2 = (p2.getY() - p1.getY()) * (p4.getX() - p2.getX()) - 
	                    (p4.getY() - p2.getY()) * (p2.getX() - p1.getX());

	// Calculate the orientation of triplet (p3, p4, p1)
	auto orientation3 = (p4.getY() - p3.getY()) * (p1.getX() - p4.getX()) - 
	                    (p1.getY() - p4.getY()) * (p4.getX() - p3.getX());

	// Calculate the orientation of triplet (p3, p4, p2)
	auto orientation4 = (p4.getY() - p3.getY()) * (p2.getX() - p4.getX()) - 
	                    (p2.getY() - p4.getY()) * (p4.getX() - p3.getX());

	// Check if orientations are different
	return (orientation1 * orientation2 < 0) && (orientation3 * orientation4 < 0);
}

/**
 * @brief Calculates the angle of the vector AB relative to the y-axis.
 * 
 * This function computes the angle between the vector formed by two points, `A` and `B`, 
 * and the y-axis in the 2D plane. The angle is measured in degrees and is calculated 
 * using trigonometric functions. The angle is computed based on the difference in the 
 * coordinates of the two points.
 * 
 * @author Anton Haes
 *
 * @tparam T The type of the coordinates (e.g., `int`, `float`, `double`). 
 *           This should be a numeric type that supports arithmetic operations and 
 *           the `std::atan` function.
 * 
 * @param A The starting point of the vector.
 * @param B The ending point of the vector.
 * 
 * @return The angle in degrees between the vector AB and the y-axis. The angle is 
 *         measured counterclockwise from the y-axis.
 */
template<typename T>
double calculateAngle(Point<T> A, Point<T> B) {
	T dx = B.getX() - A.getX();
	T dy = B.getY() - A.getY();
	double angle = 90.0;
	
	if (dx != 0) {
		if (B.getX() > 0) {
			angle = 180 - std::atan(dy/dx) * 180 / 141592653589793;
		} else {
			angle = std::atan(dy/-dx) * 180 / 3.141592653589793;
		}
	}
	
	return angle;
}

/**
 * @class PathFinding
 * @brief A class to find the path the car should follow given the detected cones.
 * 
 * This class allows the car to calculate the path the car should follow given the
 * detected cones. It also allows to calculate the direction the car should follow.
 *
 * @author Anton Haes
 */
class PathFinding {
public:
	
	std::vector<Edge<double>> edges; // All the edges of the track
	std::vector<Point<int>> path; // All the points on the path

	// Constructor for the class PathFinding
    PathFinding() {}
    
    // Destructor for the class PathFinding
    ~PathFinding() {}
    
    /**
	 * @brief Finds a path based on the given cones.
	 * 
	 * This function finds the path the car should follow given the cones. It achieves this
	 * by triangulating all the cones, removing unnecessary edges from this triangulation,
	 * and finally calculating the path. The result is stored in the path vector of this class.
	 *
	 * @author Anton Haes
	 *
	 * @param cones A pointer to a vector containing `Cone` objects. This vector represents the 
	 *              detected cones and is used as input for the triangulation and pathfinding 
	 *              process.
	 * 
	 * @note The world_coordinates_mm field of the cone objects should already be calculated.
	 */
    void findPath(std::vector<Cone>* cones) {
    	triangulate(cones);
    	filterTriangleEdges();
    	findPath();
    }
    
    /**
	 * @brief Calculate the direction the car should drive in.
	 * 
	 * This function calculates all the angles between each 'section' of the path, as well as the length
	 * of each section. This is then used to calculate the angle at which the car should point it wheels
	 * in order to follow the path.
	 *
	 * @author Anton Haes
	 * 
	 * @return The angle the car should point its front wheels at.
	 */
    uint8_t calculateDirection() {
    	// Calculate the angles between all the line sections of the path
    	// Calculate the lenght of each line section of the path
    	std::vector<double> angles;
		std::vector<double> lengths;
    	for (int i = 1; i < path.size(); i++) {
    		double angle_i = calculateAngle(path[i-1], path[i]) - 90.0;
    		if (angle_i < 90) angle_i += 180;
			if (angle_i > 90) angle_i -= 180;
			angles.push_back(angle_i);
			lengths.push_back(distance(path[i-1], path[i]));
    	}
    	
    	// Caculate angle the car should go to
    	double angle = 0.0;
    	angle = angles[0]*1.0;
		//angle += angles[0] * 0.8;
		//angle += angles[1] * 0.4;
		angle += 90.0;
		
		return (uint8_t)angle;
    }

private:
	std::vector<Point<double>> points; // This vector contains the coordinates of all the cones
	std::vector<double> coordinates; // This vector will be used by the Delaunator library
	std::vector<std::size_t> triangles; // This vector contains all the triangles made by the Delaunator library
    
    /**
	 * @brief Triangulate all the cones
	 * 
	 * This function triangulates all the cones, using Delaunay triangulation. The Delaunator library is
	 * used for this purpose.The result is stored in the triangles vector from this class.
	 *
	 * @author Anton Haes
	 */
    void triangulate(std::vector<Cone>* cones) {
    	// Make sure all the vectors are empty
    	points.clear();
    	coordinates.clear();
    	triangles.clear();
    	edges.clear();
		// Populate the vectors
		for (int i = 0; i < cones->size(); i++) {
			double x = (double)(*cones)[i].world_coordinates_mm[0];
			double y = (double)(*cones)[i].world_coordinates_mm[1];
			coordinates.emplace_back(x);
			coordinates.emplace_back(y);
			int color = (*cones)[i].type;
			points.emplace_back(Point(x, y, color));
		}

		delaunator::Delaunator delaunay(coordinates); // Triangulate the points
		triangles = delaunay.triangles; // Save the triangulation
	}
	
	/**
	 * @brief Filter the edges of each triangle to eliminate unnecessary ones.
	 * 
	 * This function filters the edges of each triangle, only keeping the edges
	 * which form the track boundaries or go over the middle of the track. The
	 * latter ones are used later to calculate the path. The results are stored
	 * in the edges vector from this class.
	 *
	 * @author Anton Haes
	 */
	void filterTriangleEdges() {
		for(std::size_t i = 0; i < triangles.size(); i+=3) {
			// Get the 3 points from the triangle
			double point1_x = coordinates[2 * triangles[i + 0]];
			double point1_y = coordinates[2 * triangles[i + 0] + 1];
			double point2_x = coordinates[2 * triangles[i + 1]];
			double point2_y = coordinates[2 * triangles[i + 1] + 1];
			double point3_x = coordinates[2 * triangles[i + 2]];
			double point3_y = coordinates[2 * triangles[i + 2] + 1];
			
			// Find the index of triangle points in the vector points
			std::vector<Point<double>>::iterator iterator_point1 = std::find(points.begin(), points.end(), Point(point1_x, point1_y));
			std::vector<Point<double>>::iterator iterator_point2 = std::find(points.begin(), points.end(), Point(point2_x, point2_y));
			std::vector<Point<double>>::iterator iterator_point3 = std::find(points.begin(), points.end(), Point(point3_x, point3_y));
			int index_point1 = std::distance(points.begin(), iterator_point1);
			int index_point2 = std::distance(points.begin(), iterator_point2);
			int index_point3 = std::distance(points.begin(), iterator_point3);
			
			// Reconstruct the points (these now also have color information)
			Point point1 = points[index_point1];
			Point point2 = points[index_point2];
			Point point3 = points[index_point3];

			// We are not interested by triangles with all points of the same color
			if ((int)(point1.color==point2.color) + (int)(point1.color==point3.color) + (int)(point2.color==point3.color) != 3) {
				// We keep the edge connecting the 2 points with the same color
				// From the other 2 edges, we keep the shortest one
				if (point1.color == point2.color) {
					edges.push_back(Edge(point1, point2, point1.color==point2.color));
					if (distance(point1, point3) < distance(point2, point3)) {
						edges.push_back(Edge(point1, point3, point1.color==point3.color));
					} else {
						edges.push_back(Edge(point2, point3, point2.color==point3.color));
					}
				} else if (point1.color == point3.color) {
					edges.push_back(Edge(point1, point3, point1.color==point3.color));
					if (distance(point1, point2) < distance(point2, point3)) {
						edges.push_back(Edge(point1, point2, point1.color==point2.color));
					} else {
						edges.push_back(Edge(point2, point3, point2.color==point3.color));
					}
				} else if (point2.color == point3.color) {
					edges.push_back(Edge(point2, point3, point2.color==point3.color));
					if (distance(point1, point2) < distance(point1, point3)) {
						edges.push_back(Edge(point1, point2, point1.color==point2.color));
					} else {
						edges.push_back(Edge(point1, point3, point1.color==point3.color));
					}
				}
			}
		}
    }
    
    /**
	 * @brief Find the path the car should follow
	 * 
	 * This function finds points which are on the path the car should follow. It then
	 * sorts these points in order of appearence for the car. The result is a vector of
	 * points, all of which lie on the path the car should follow. This result is stored
	 * in the path vector from this class.
	 *
	 * @author Anton Haes
	 */
    void findPath() {
    	// Vectors for the (un)sorted path_points. The first point of the path is always (0, 0)
		std::vector<Point<int>> path_points_unsorted;
		std::vector<Point<int>> path_points_sorted;
		path_points_sorted.push_back(Point(0, 0));
		// This set is used to avoid duplicates in path_points_unsorted
		std::set<Point<int>> unique_points;
		
		// Find all the points in the middle of the track
		for (Edge<double> edge: edges) {
			// A point on the middle of the track always lies on an edge connecting 2 points of different colors
			if (!edge.same_color) {
				// Find the middle of that edge
				double x = (edge.point1.getX()+edge.point2.getX())/2;
				double y = (edge.point1.getY()+edge.point2.getY())/2;
				Point<int> point((int)x, (int)y);
				
				// Check if the point is already in the set, in order to avoid duplicates
				if (unique_points.find(point) == unique_points.end()) {
				    // If not found, add it to both the vector and the set
				    path_points_unsorted.push_back(point);
				    unique_points.insert(point);
				}
			}
		}
		int number_of_points = path_points_unsorted.size();
		
		// sort these points in order of appearance
		while (path_points_sorted.size() < number_of_points + 1) {
			Point<int> last_sorted_point = path_points_sorted.back();
			// Find the point which is closest to last_sorted_point
			std::vector<Point<int>>::iterator iterator_next_point = std::min_element(path_points_unsorted.begin(), path_points_unsorted.end(),
			                            [last_sorted_point](const Point<int>& p1, const Point<int>& p2) -> bool {
			                                return compareDistance(p1, p2, last_sorted_point);
			                            });
			Point<int> next_point = *iterator_next_point;
			
			// If the edge between last_sorted_point and next_point intersects the track boundaries, we have reached the end of the path
			bool intersects_track_edge = false;
			for (Edge<double> edge: edges) {
				// An edge is a track boundary if it connects 2 points of the same color
				if (edge.same_color) {
					Point<double> boundary1 = edge.point1;
					Point<double> boundary2 = edge.point2;
					if (intersects(next_point, last_sorted_point, boundary1, boundary2)) {
						intersects_track_edge = true;
						break;
					}
				}
			}
			if (intersects_track_edge) {
				break;
			}
			
			path_points_sorted.push_back(next_point);
			path_points_unsorted.erase(iterator_next_point);
		}

		path = path_points_sorted;
    }
    
    
};

#endif // PATH_FINDING_HPP

