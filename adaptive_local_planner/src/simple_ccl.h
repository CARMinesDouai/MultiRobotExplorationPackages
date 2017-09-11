#ifndef SIMPLE_CCL_H
#define SIMPLE_CCL_H

//#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;


typedef struct{
    int x;
    int y;
    int label;
    int value;
} cell_t;

typedef struct 
{
    int parent;
    int rank;
    int cnt;
}subset_t;

class SimpleCCL {
public:
    SimpleCCL();
    ~SimpleCCL();
    void setMap(nav_msgs::OccupancyGrid map);
    void print();
    int dw, dh;
    vector<int8_t>data;
    set<int> labels;
    vector<subset_t> labels_tree;
private:
    void ccl(nav_msgs::OccupancyGrid map);
    vector<cell_t> neighbors_of(cell_t curr, nav_msgs::OccupancyGrid map, bool);
    int label_find(int);
    void label_union(int, int);
    int min_label_of(vector<cell_t>);

};

#endif