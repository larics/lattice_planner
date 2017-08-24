#ifndef MAT_BY_NAME // fix issue with different versions of matio library
    #define MAT_BY_NAME BY_NAME
#endif

#include <fstream>
#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <matio.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include "defines.h"

#include <agv_control_msgs/GetMyPlan.h>
#include <agv_control_msgs/planData.h>

using namespace std;

struct lattice_params {
    matvar_t * vars;
    matvar_t * cost;
    matvar_t * edges;
    matvar_t * stop;
    matvar_t * dist;
} latt_coarse, latt_fine;

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

struct boundary_pose {
    int x;
    int y;
    int fi;
} bp_start, bp_goal;

vector<boundary_pose> boundary_poses;

short* originalMap = NULL;
short* myMap = NULL;
int mapWidth, mapHeight = 0;
float mapResolution = 0;
int node_dist = 5;
ros::Publisher plan_pub, goal_pub;
 
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;
std::string robot_frame_id = "robot";
std::string map_frame_id = "map";

vector<agv_control_msgs::GetMyPlan> my_prev_plans;

void graphSearch(matvar_t * varVars, matvar_t * varCost, const vector<vector<short> >& map, int start[], int finish[], vector<int> wrX, vector<int> wrY, double** M, int sizeM[], int** veh_orientations);
void generate_path_data(double* M, int numOfSeg, lattice_params &lattice, agv_control_msgs::planData &plan, int start[]);
bool load_previous_plan(agv_control_msgs::GetMyPlan::Request &req, agv_control_msgs::GetMyPlan::Response &res);
void get_border_cells(const vector<pair<int, int> >& full_vector, vector<pair<int, int> >& reduced_vector);

// ******************************************************************
// ****** DISCRETIZATION OF THE VEHICLE POSITION & ORIENTATION ******
// ******************************************************************
// orientation: [0, 2*PI] -> [0, 16]
int c2d_yaw( double r )
{
    r /= M_PI/8;
    int fi = (r > 0.0) ? (r + 0.5) : (r - 0.5);

    while (fi < 0)  fi += 16;
    while (fi > 15) fi -= 16;
    
    return fi;
}

int c2d_pose( double r)
{
    return (r > 0.0) ? (r + (float)node_dist/2) / node_dist : (r - (float)node_dist/2) / node_dist;
}
// ******************************************************************


// ************************** CALLBACKS *****************************

/** Callback invoked on map received event */
void mapReceivedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapWidth =  msg->info.width;
    mapHeight = msg->info.height;

    mapResolution = msg->info.resolution; // [m/cell]

    originalMap = new short[mapWidth * mapHeight * sizeof(short)];

    for (int i = 0; i < mapWidth * mapHeight; i++)
    {
        if (msg->data[i] != 0)      // (msg->data[i] == 100 || msg->data[i] == -1)  -> occupied or unknown cell in OccupancyGrid message
            originalMap[i] = -1;    // occupied cell in pathPlanner's map representation
        else                        // (msg->data[i] == 0) -> unoccupied cell in OccupancyGrid message
            originalMap[i] = 1;     // unoccupied cell in pathPlanner's map representation
            
    }

    ROS_INFO("pathPlanner received a %d X %d map @ %.3f m/pix", mapWidth, mapHeight, mapResolution);
}

/** Callback invoked on getPath request */
bool getPlanCallback(agv_control_msgs::GetMyPlan::Request &req, agv_control_msgs::GetMyPlan::Response &res)
{
    if(load_previous_plan(req, res))
        return true;
        
    if(originalMap != NULL)
    {
        myMap = new short[mapWidth * mapHeight];
        memcpy(myMap, originalMap, mapWidth * mapHeight * sizeof(short));
        const double* data = static_cast<const double*>(latt_coarse.dist->data);
        node_dist = (int)data[0];
        
        tf::Quaternion q1;
        
        if(req.forbiddenCells.xGrid.size() > 0)
        {
            // zabranjene čelije ucrtavaju se u kartu prostora kao prepreke
            for (uint32_t i = 0; i < req.forbiddenCells.xGrid.size(); i++)
            {
                for (uint32_t j = 0; j < req.forbiddenCells.xGrid[i].data.size(); j++)
                {
                    myMap[(int)req.forbiddenCells.xGrid[i].data[j] + ((int)req.forbiddenCells.yGrid[i].data[j]) * mapWidth] = -1;
                }
            }
        }
        
        // učitavanje koordinata i orijentacije vozila koje predstavlja prepreku
        if(req.removing)
        {
            // diskretizacija prostora
            int carx = c2d_pose(req.carobst.pose.position.x / mapResolution);
            int cary = c2d_pose(req.carobst.pose.position.y / mapResolution);

            tf::quaternionMsgToTF(req.carobst.pose.orientation, q1);
            int carfi = c2d_yaw(tf::getYaw(q1));

            matvar_t* matvar = Mat_VarGetCell(latt_coarse.stop, carfi);
            double* carstop = (double*)matvar->data;

            int m = matvar->dims[0];

            // ucrtavanje prepreke u kartu prostora
            for (int i = 0; i < m; i++)
            {
                myMap[(carx + (int)carstop[i]) + (cary + (int)carstop[i + m]) * mapWidth] = -1;
            }
        }
        
        vector<vector<short> > vMap;    // vektorski zapis karte
        vector<short> dtm;
        for (int i = 0; i < mapWidth; i++)
        {
            dtm.clear();
            for (int j = 0; j < mapHeight; j++)
            {
                dtm.push_back(myMap[i + j * mapWidth]);
            }  
            vMap.push_back(dtm);
        }
        
        delete myMap;
        
        int xStart, yStart, xGoal, yGoal, fiStart, fiGoal;
        double startYaw, goalYaw;
        
        boundary_poses.clear();
        
        // initial vehicle pose on the path
        bp_start.x = c2d_pose(req.start.pose.position.x / mapResolution);
        bp_start.y = c2d_pose(req.start.pose.position.y / mapResolution);
        tf::quaternionMsgToTF(req.start.pose.orientation, q1);
        startYaw = tf::getYaw(q1);
        bp_start.fi = c2d_yaw(startYaw);
        
        // final vehicle pose on the path
        bp_goal.x = c2d_pose(req.goal.pose.position.x / mapResolution);
        bp_goal.y = c2d_pose(req.goal.pose.position.y / mapResolution);
        tf::quaternionMsgToTF(req.goal.pose.orientation, q1);
        goalYaw = tf::getYaw(q1);
        bp_goal.fi = c2d_yaw(goalYaw);
        
        // dodavanje početnog i konačnog položaja u vektor graničnih točaka putanje
        boundary_poses.push_back(bp_start);
        boundary_poses.push_back(bp_goal);
        
        double* M;
        
        // vektor "veh_orientations" sadrži stvarne orijentacije vozila na početku svakog segmenta, počevši od zadnjeg prema prvom
        int* veh_orientations = NULL;

        // petlja u kojoj se određuju sve komponente putanje (po pojedinim sektorima)
        for (vector<boundary_pose>::iterator it = boundary_poses.begin(); it != (boundary_poses.end() - 1); it++)
        {
            int start[] =  {it->x, it->y, it->fi};
            int finish[] = {(it + 1)->x, (it + 1)->y, (it + 1)->fi};
        
            if(start[0] == finish[0] && start[1] == finish[1] && start[2] == finish[2])
                ROS_WARN("Identical start and goal poses!\n");

            int sizeM[2];
            M = NULL;
            
            // traženje najkraće izvedive putanje na grafu latice korištenjem A* algoritma
            graphSearch(latt_coarse.vars, latt_coarse.cost, vMap, start, finish, req.wrongX, req.wrongY, &M, sizeM, &veh_orientations);
    
            // **********************************************************************************************************************
            // Ako putanja nije uspješno isplanirana (npr. zato što u originalnom startnom čvoru vozilo zahvaća neku blisku prepreku)
            // pokušava se isplanirati sa startnom pozicijom u jednom od 8 susjednih čvorova:
            // **********************************************************************************************************************
            if(!(start[0] == finish[0] && start[1] == finish[1] && start[2] == finish[2]))
            {
                vector<pair<int, int> > neighbors;  // parovi x-y koordinata svih susjednih čvorova
            
                if(start[0] + 1 < mapWidth / node_dist)
                {
                    neighbors.push_back(make_pair(start[0] + 1, start[1]));
                }
                else if(start[0] - 1 > 0)
                {
                    neighbors.push_back(make_pair(start[0] - 1, start[1]));
                }
                else if(start[1] + 1 < mapHeight / node_dist)
                {
                    neighbors.push_back(make_pair(start[0], start[1] + 1));
                }
                else if(start[1] - 1 > 0)
                {
                    neighbors.push_back(make_pair(start[0], start[1] - 1));
                }
                else if((start[0] + 1 < mapWidth / node_dist) && (start[1] + 1 < mapHeight / node_dist))
                {
                    neighbors.push_back(make_pair(start[0] + 1, start[1] + 1));
                }
                else if((start[0] + 1 < mapWidth / node_dist) && (start[1] - 1 > 0))
                {
                    neighbors.push_back(make_pair(start[0] + 1, start[1] - 1));
                }
                else if((start[0] - 1 > 0) && (start[1] + 1 < mapHeight / node_dist))
                {
                    neighbors.push_back(make_pair(start[0] - 1, start[1] + 1));
                }
                else if((start[0] - 1 > 0) && (start[1] - 1 < 0))
                {
                    neighbors.push_back(make_pair(start[0] - 1, start[1] - 1));
                }
            
                int index = 0;

                while(!(M != NULL && sizeM[1] > 0) && index < neighbors.size())
                {
                    start[0] = neighbors[index].first;
                    start[1] = neighbors[index].second;
                
                    graphSearch(latt_coarse.vars, latt_coarse.cost, vMap, start, finish, req.wrongX, req.wrongY, &M, sizeM, &veh_orientations);
                
                    index++;
                }
            }
            // **********************************************************************************************************************
        
            int numOfSegments = sizeM[1];
        
            if(M != NULL && numOfSegments > 0)
            {   
                if(latt_fine.vars == NULL || req.removing)  // ako se ne koristi planiranje s dvije latice ILI se radi o removingu
                {
                    generate_path_data(M, numOfSegments, latt_coarse, res.plan, start);
                }
                else
                {
                    // *******************************************************************************************************
                    //  Replaniranje završnog dijela putanje pomoću fine latice zbog postizanja veće točnosti pozicioniranja
                    // *******************************************************************************************************
                    if(numOfSegments > 3)
                    {
                        generate_path_data(M, numOfSegments - 3, latt_coarse, res.plan, start);
                    
                        // učitavanje "node_dist" parametra iz fine latice
                        const double* data = static_cast<const double*>(latt_fine.dist->data);
                        node_dist = (int)data[0];
                    
                        start[0] = c2d_pose(res.plan.xp[res.plan.xp.size() - 1]);
                        start[1] = c2d_pose(res.plan.yp[res.plan.yp.size() - 1]);
                        start[2] = veh_orientations[2]; // orijentacija vozila na trenutnoj putanji u čvoru udaljenom 3 segmenta od cilja
                    }
                    else
                    {
                        start[0] = c2d_pose(req.start.pose.position.x / mapResolution);
                        start[1] = c2d_pose(req.start.pose.position.y / mapResolution);
                    }
    
                    // određivanje ciljnog čvora
                    finish[0] = c2d_pose(req.goal.pose.position.x / mapResolution);
                    finish[1] = c2d_pose(req.goal.pose.position.y / mapResolution);
                
                    // PLANIRANJE POMOĆU FINE LATICE
                    graphSearch(latt_fine.vars, latt_fine.cost, vMap, start, finish, req.wrongX, req.wrongY, &M, sizeM, &veh_orientations);
                    
                    generate_path_data(M, sizeM[1], latt_fine, res.plan, start);
                    // *******************************************************************************************************
                }
            }
            else
            {
                res.plan.xp.clear();
                res.plan.yp.clear();
                res.plan.fip.clear();
                res.plan.sp.clear();

                res.plan.grid.xGrid.clear();
                res.plan.grid.yGrid.clear();
                res.plan.grid.timeInGrid.clear();
                res.plan.grid.timeOutGrid.clear();
            
                break;
            }
        }
        
        res.plan.header = req.start.header;
        res.plan.header.stamp = ros::Time::now();
        
        agv_control_msgs::GetMyPlan srv;
        srv.request = req;
        srv.response = res;
        
        my_prev_plans.push_back(srv);
        
        delete M;
        delete veh_orientations;
    }
    else
    {
        ROS_ERROR("Unable to plan the path.. pathPlanner node has not received the map!");
        return false;
    }

    return true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  agv_control_msgs::GetMyPlan::Request req;

  // Get start pose from /tf topic
  geometry_msgs::TransformStamped transform;
  bool transform_ok = true;
  try
  {
    transform = tfBuffer->lookupTransform(map_frame_id, robot_frame_id,
                                          ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    transform_ok = false;
  }

  if (transform_ok)
  {
    req.start.header.stamp = transform.header.stamp;
    req.start.header.frame_id = map_frame_id;
    req.start.pose.position.x = transform.transform.translation.x;
    req.start.pose.position.y = transform.transform.translation.y;
    req.start.pose.position.z = transform.transform.translation.z;
    req.start.pose.orientation = transform.transform.rotation;

    // TODO: Check that msg.header.frame_id correspondst to map_frame_id
    req.goal = *msg;
    req.removing = false;
    
    agv_control_msgs::GetMyPlan::Response res;
    if (getPlanCallback(req, res))
    {
      // Planning was successful, extract the path and publish it
      nav_msgs::Path plan;
      plan.header.stamp = ros::Time::now();
      plan.header.frame_id = req.goal.header.frame_id;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = req.goal.header.frame_id;
      for (uint32_t i = 0; i < res.plan.xp.size(); i++)
      {
        pose.header.seq = i;
        pose.pose.position.x = mapResolution * res.plan.xp[i];
        pose.pose.position.y = mapResolution * res.plan.yp[i];
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw( res.plan.fip[i] );
        plan.poses.push_back(pose);
      }
      plan_pub.publish(plan);
      goal_pub.publish(plan.poses.back());
    }
  }
}

// ******************************************************************

// ******************************************************************
void generate_path_data(double* M, int Mn, lattice_params &lattice, agv_control_msgs::planData &plan, int start[])
{
    matvar_t *dx, *dy, *x, *y, *fi, *sf, *cost;

    char var1Name[] = "dx";
    char var2Name[] = "dy";
    char var3Name[] = "x";
    char var4Name[] = "y";
    char var5Name[] = "fi";
    char var6Name[] = "sf";
    char var7Name[] = "cost";
    int k = 0;
    
    int tx = node_dist * start[0];
    int ty = node_dist * start[1];
    double ts = 0;
    
    if(plan.sp.size() > 0)
        ts = plan.sp[plan.sp.size() - 1];
            
    for(int i = 0; i < Mn; i++)
    {
        int struct_lin_index = M[k] + M[k+1]*(lattice.edges->dims[0]); // M[k] -> indeks retka u "edges", M[k+1] -> indeks stupca

        dx = Mat_VarGetStructField(lattice.edges, var1Name, MAT_BY_NAME, struct_lin_index);
        dy = Mat_VarGetStructField(lattice.edges, var2Name, MAT_BY_NAME, struct_lin_index);
         x = Mat_VarGetStructField(lattice.edges, var3Name, MAT_BY_NAME, struct_lin_index);
         y = Mat_VarGetStructField(lattice.edges, var4Name, MAT_BY_NAME, struct_lin_index);

        fi = Mat_VarGetStructField(lattice.edges, var5Name, MAT_BY_NAME, struct_lin_index);
        sf = Mat_VarGetStructField(lattice.edges, var6Name, MAT_BY_NAME, struct_lin_index);
        cost = Mat_VarGetStructField(lattice.edges, var7Name, MAT_BY_NAME, struct_lin_index);

        double* dxData = static_cast<double*>(dx->data);
        double* dyData = static_cast<double*>(dy->data);
        double*  xData = static_cast<double*>( x->data);
        double*  yData = static_cast<double*>( y->data);
        double* fiData = static_cast<double*>(fi->data);
        double* sfData = static_cast<double*>(sf->data);
        double*  CData = static_cast<double*>(cost->data);
                
        int xm = x->dims[1]; // broj točaka na svakom segmentu putanje
        for(int j = 0; j < xm; j++)
        {
            plan.xp.push_back( xData[j] + tx );
            plan.yp.push_back( yData[j] + ty );
            plan.fip.push_back( fiData[j] );
            plan.sp.push_back( sfData[0]/(xm-1) * j + ts );
        }

        int Cm = cost->dims[0];

        agv_control_msgs::arrayData gDataX, gDataY, gDataTI, gDataTO;

        for(int j = 0; j < Cm; j++)
        {
            gDataX.data.push_back( CData[j] + tx );
            gDataY.data.push_back( CData[Cm + j] + ty );
            gDataTI.data.push_back( CData[2*Cm + j] );
            gDataTO.data.push_back( CData[3*Cm + j] );
        }
        
        // *****************************************************
        // reduciranje vektora s ćelijama na samo vanjske ćelije
        // *****************************************************
        pair<int, int> cell;
        vector<pair<int, int> > full, reduced;
        for(uint32_t j = 0; j < gDataX.data.size(); j++)
        {
            cell = make_pair(gDataX.data[j], gDataY.data[j]);
            full.push_back(cell);
        }
        
        get_border_cells(full, reduced);
        
        gDataX.data.clear();
        gDataY.data.clear();
        for(uint32_t j = 0; j < reduced.size(); j++)
        {
            gDataX.data.push_back(reduced[j].first);
            gDataY.data.push_back(reduced[j].second);
        }
        // *****************************************************
        
        if(i == Mn - 1)
        {
            double N = gDataTO.data[0];

            for(uint32_t j = 0; j < gDataTO.data.size(); j++)
            {
                if(gDataTO.data[j] > N)
                    N = gDataTO.data[j];
            }

            for(uint32_t j = 0; j < gDataTO.data.size(); j++)
            {
                if(abs(gDataTO.data[j] - N) < 0.001)
                    gDataTO.data[j] = 10000;
            }
        }
        
        plan.grid.xGrid.push_back(gDataX);
        plan.grid.yGrid.push_back(gDataY);
        plan.grid.timeInGrid.push_back(gDataTI);
        plan.grid.timeOutGrid.push_back(gDataTO);

        tx += dxData[0];
        ty += dyData[0];
        ts += sfData[0];

        k += 2;
    }
}
// ******************************************************************

bool load_previous_plan(agv_control_msgs::GetMyPlan::Request &req, agv_control_msgs::GetMyPlan::Response &res)
{
    vector<agv_control_msgs::GetMyPlan>::iterator it;
    
    for(it = my_prev_plans.begin(); it != my_prev_plans.end(); it++) 
    {
        if(c2d_pose(req.start.pose.position.x / mapResolution) == c2d_pose(it->request.start.pose.position.x / mapResolution))
            if(c2d_pose(req.start.pose.position.y / mapResolution) == c2d_pose(it->request.start.pose.position.y / mapResolution))
                if(c2d_pose(req.goal.pose.position.x / mapResolution) == c2d_pose(it->request.goal.pose.position.x / mapResolution))
                    if(c2d_pose(req.goal.pose.position.y / mapResolution) == c2d_pose(it->request.goal.pose.position.y / mapResolution))
                    {
                        res = it->response;
                        res.plan.header = req.start.header;
                        res.plan.header.stamp = ros::Time::now();
                        
                        fprintf(stderr, BOLDBLUE "pathPlanner   - Using chached path data.\n" RESET);
                        return true;
                    }
    }
    
    return false;
}

void get_border_cells(const vector<pair<int, int> >& full_vector, vector<pair<int, int> >& reduced_vector)
{
    for(uint32_t j = 0; j < full_vector.size(); j++)
    {
        pair<int, int> test_cell = full_vector[j];
        
        bool has_all_neighbours = true;
        
        // check whether the test_cell has all neighbours 
        for(int k = -1; k <= 1; k++)
        {
            for(int m = -1; m <= 1; m++)
            {
                if(!(k == 0 && m == 0))
                {
                    pair<int, int> neighbour_cell = make_pair(test_cell.first + k, test_cell.second + m);
                    
                    if(std::find(full_vector.begin(), full_vector.end(), neighbour_cell) == full_vector.end())
                    {
                        has_all_neighbours = false;
                        break;
                    }
                }
            }
            if(!has_all_neighbours) 
            {
                reduced_vector.push_back(test_cell);
                break;
            }
        }
    }
}

// ********************** LOADING ".mat" file ***********************
bool load_MAT_file(const char *file_name, lattice_params &lattice)
{
    mat_t *matfp;

    char var1Name[] = "vars";   // matrica čiji retci povezuju početnu orijentaciju, koord. susjednog čvora, konačnu orijentaciju i cijenu pripadajućeg segmenta
    char var2Name[] = "cost";   // polje cell-ova koji sadrže koordinate svih čelija kroz koje vozilo prolazi prilikom gibanja pojedinim segmentom putanje
    char var3Name[] = "edges";  // matrica cell-ova koji sadrže sljedeće informacije o svakom segmentu latice: dx, dy, fif, x, y, fi, sf, inTime, outTime, R, cost.
    char var4Name[] = "stop";   // polje cell-ova koji sadrže koordinate prostornih čelija koje vozilo okupira u mirovanju pri pojedinoj orijentaciji
    char var5Name[] = "dist";   // podatak o udaljenosti između čvorova u latici (u jediničnim duljinama)

    matfp = Mat_Open(file_name, MAT_ACC_RDONLY);

    if ( matfp == NULL )
    {
        ROS_FATAL("Error opening MAT file \"%s\"!", file_name);
        return false;
    }

    lattice.vars = Mat_VarRead(matfp, var1Name);

    if ( lattice.vars == NULL )
    {
        ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var1Name, file_name);
        return false;
    }

    //Mat_VarPrint( lattice.vars, 1);

    lattice.cost = Mat_VarRead(matfp, var2Name);

    if ( lattice.cost == NULL )
    {
        ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var2Name, file_name);
        return false;
    }

    //Mat_VarPrint( lattice.cost, 1);

    lattice.edges = Mat_VarRead(matfp, var3Name);

    if ( lattice.edges == NULL )
    {
        ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var3Name, file_name);
        return false;
    }

    //Mat_VarPrint( lattice.edges, 1);

    lattice.stop = Mat_VarRead(matfp, var4Name);

    if ( lattice.stop == NULL )
    {
        ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var4Name, file_name);
        return false;
    }

    //Mat_VarPrint( lattice.stop, 1);

    lattice.dist = Mat_VarRead(matfp, var5Name);

    if ( lattice.dist == NULL )
    {
        ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var5Name, file_name);
        return false;
    }
    
    // ******************************************************************

    Mat_Close(matfp);
    
    return true;
}

static void show_usage(std::string name)
{
    std::cerr << "\nUsage: " << name << " [option] [argument]\n\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-cl COARSE_LATTICE_FILE_PATH\tSpecify the coarse lattice file path\n"
              << "\t-fl FINE_LATTICE_FILE_PATH\tSpecify the fine lattice file path\n\n"
              << "Legacy usage: " << name << " COARSE_LATTICE_FILE_PATH <FINE_LATTICE_FILE_PATH>\n"
              << std::endl;
}
    
int main(int argc, char **argv)
{
    if (argc < 1) 
    {
        ROS_FATAL("pathPlanner: Not enough parameters!\n");
        show_usage(argv[0]);
        return 0;
    }
    
    string cl_file_path = "";       // coarse lattice MAT file path
    string fl_file_path = "";       // fine lattice MAT file path
    
    for (int i = 1; i < argc; ++i) 
    {
        string arg = argv[i];
        std::cout << "argv[" << i << "] = " << arg << std::endl;
        if ((arg == "-h") || (arg == "--help")) 
        {
            show_usage(argv[0]);
            return 0;
        } 
        else if (arg == "-cl") 
        {
            if (i + 1 < argc) 
            { 
                cl_file_path = argv[++i];
            } 
            else
            {
                std::cerr << "-cl option requires one argument." << std::endl;
                return 0;
            }  
        }
        else if (arg == "-fl") 
        {
            if (i + 1 < argc) 
            { 
                fl_file_path = argv[++i];
            } 
            else
            {
                std::cerr << "-fl option requires one argument." << std::endl;
                return 0;
            }
        }
        else
        {
          // Handle positional arguments
          if (i == 1)
          {
            cl_file_path = argv[i];
          }
          else if (i == 2)
          {
            fl_file_path = argv[i];
          }
        }
    }
    
    ros::init(argc, argv, "pathPlanner");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    if(load_MAT_file(cl_file_path.c_str(), latt_coarse) == false)   // loading latt_coarse params
        return 0;

    if(argc > 2 && fl_file_path != "")
        if(load_MAT_file(fl_file_path.c_str(), latt_fine) == false) // loading latt_fine params
            return 0;
            
    // Subscriber za primanje karte prostora:
    ros::Subscriber subMap = n.subscribe("map", 1, mapReceivedCallback);
    
    ros::ServiceServer getPathService = n.advertiseService("getPlanSrv", getPlanCallback);

    // New interface, (partially) conforming to nav_core spec
    
    // Subscriber for receiving planner goals
    ros::Subscriber goal_sub = n.subscribe("goal", 1, goalCallback);
    // Publisher for publishig the path
    plan_pub = n.advertise<nav_msgs::Path>("plan", 1);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("planned_goal", 1);
      
    // Read tf frame parameters
    nh.param<std::string>("map_frame_id", map_frame_id, "map");
    nh.param<std::string>("robot_frame_id", robot_frame_id, "robot");
    
    // The tf listener is initialized from global pointers,
    // as it seems to require that the node has been initialized beforehand
    // This is absolutely horrible, an should be fixed asap.
    tfBuffer = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    
    ros::spin();

    delete originalMap;
    
    Mat_VarFree(latt_coarse.vars);
    Mat_VarFree(latt_coarse.cost);
    Mat_VarFree(latt_coarse.edges);
    Mat_VarFree(latt_coarse.stop);

    return 0;
}
