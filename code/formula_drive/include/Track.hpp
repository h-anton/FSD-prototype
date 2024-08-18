#ifndef TRACK_HPP
#define TRACK_HPP

/**
 * @brief A template structure representing a 2D point with a color attribute.
 * 
 * @author Anton Haes
 * 
 * @tparam T The data type for the x and y coordinates (e.g., int, float, double).
 */
template<typename T>
struct Point {
	/**
     * @brief A pair representing the 2D coordinates of the point (x, y).
     */
	std::pair<T, T> point;
	/**
     * @brief The color of the point, represented as an integer.
     */
	int color;
	
	/**
     * @brief Default constructor initializing the point at (0, 0) with no color.
     */
	Point() : point{0, 0, -1} {}
	
	/**
     * @brief Constructor initializing the point with specified coordinates and no color.
     * 
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     */
	Point(T x, T y) : point(x, y) {}
	
	/**
     * @brief Constructor initializing the point with specified coordinates and color.
     * 
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     * @param c The color of the point.
     */
	Point(T x, T y, int c) : point(x, y), color(c) {}
	
	/**
     * @brief Get the x-coordinate of the point.
     * 
     * @return The x-coordinate.
     */
	T getX() const {
		return point.first;
	}
	
	/**
     * @brief Get the y-coordinate of the point.
     * 
     * @return The y-coordinate.
     */
	T getY() const {
		return point.second;
	}
	
	/**
     * @brief Equality operator to compare two points.
     * 
     * Two points are considered equal if both their x and y coordinates are the same.
     * 
     * @param other The other point to compare with.
     * @return True if the points are equal, false otherwise.
     */
    bool operator==(const Point& other) const {
        return getX() == other.getX() && getY() == other.getY();
    }
    
    /**
     * @brief Less-than operator to order points (used by containers like std::set).
     * 
     * @param other The other point to compare with.
     * @return True if this point is less than the other point, false otherwise.
     */
    bool operator<(const Point& other) const {
        if (getX() == other.getX())
            return getY() < other.getY();
        return getX() < other.getX();
    }
    
    /**
     * @brief Get a string representation of the point.
     * 
     * @return A string representing the point in the format "Point(x, y, color=color)".
     */
    std::string print() const {
    	std::ostringstream oss;
        oss << "Point(" << getX() << ", " << getY() << ", color=" << color << ")";
        return oss.str();
    }
};

/**
 * @brief A template structure representing an edge connecting two 2D points with an attribute indicating if they share the same color.
 * 
 * @tparam T The data type for the coordinates of the points (e.g., int, float, double).
 */
template<typename T>
struct Edge {
	/**
     * @brief The first point of the edge.
     */
	Point<T> point1;
	/**
     * @brief The second point of the edge.
     */
	Point<T> point2;
	/**
     * @brief A boolean indicating whether the two points share the same color.
     * 
     * True if both points have the same color, false otherwise.
     */
	bool same_color;
	/**
     * @brief Constructor initializing the edge with two points and a color comparison flag.
     * 
     * @param pt1 The first point of the edge.
     * @param pt2 The second point of the edge.
     * @param sc A boolean indicating whether the points share the same color.
     */
	Edge(Point<T>& pt1, Point<T>& pt2, bool sc)
		: point1(pt1), point2(pt2), same_color(sc) {
	
	}
};

#endif // TRACK_HPP

