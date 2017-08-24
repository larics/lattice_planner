#include <math.h>
#include <limits.h>
#include "defines.h"
#include <flann/mpi/matrix.h>
#include <algorithm>
#include <vector>
#include <map>
#include <matio.h>
#include "ros/ros.h"
#include <nav_msgs/GridCells.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#define INF 1000000.0

using namespace std;
using namespace __gnu_cxx;
using namespace flann;

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

int xf, yf, fif;

extern const float mapResolution;
extern const int node_dist;

void publishVisitedNodes(nav_msgs::GridCells gCells);

class Vector : public vector<int>
{
public:
    //operator size_t() const { return (*this).size(); };
    operator size_t() const { return (*this)[0]*1600 + (*this)[1]*16 + (*this)[2]; };
};

// zamjena mjesta elemenata na indexima i,j u vektoru
void swap(int i, int j, vector<pair<Vector, double> > *v, map<Vector, int> *h)
{
    pair<Vector, double> tmp;
    tmp = (*v)[i];
    (*v)[i] = (*v)[j];
    (*v)[j] = tmp;

    (*h)[(*v)[i].first] = i;
    (*h)[(*v)[j].first] = j;
}

// podizanje elementa na odgovarajući nivo u binarnom stablu
void bubble_up(int index, vector<pair<Vector, double> > *v, map<Vector, int> *h)
{
    int new_ind;
    
    while (index > 0)
    {
        new_ind = (index - 1)/2;
        
        if ((*v)[index].second < (*v)[new_ind].second)
        {
            swap(index, new_ind, v, h);
            index = new_ind;
        } 
        else 
        {
            break;
        }
    }
}

// spuštanje elementa na odgovarajući nivo u binarnom stablu
void bubble_down(int index, vector<pair<Vector, double> > *v, map<Vector, int> *h)
{
    int size = v->size();
    int new_ind;
    
    while (true)
    {
        if (2 * index + 2 >= size)
        {
            if (2 * index + 1 >= size)
            {
                break;
            } 
            else 
            {
                new_ind = 2 * index + 1;
            }
        } 
        else 
        {
            if ((*v)[2 * index + 1].second < (*v)[2 * index + 2].second)
            {
                new_ind = 2 * index + 1;
            } 
            else 
            {
                new_ind = 2 * index + 2;
            }
        }
        
        if ((*v)[index].second > (*v)[new_ind].second)
        {
            swap(index, new_ind, v, h);
            index = new_ind;
        } 
        else 
        {
            break;
        }
    }
}

// provjera izvedivosti i određivanje cijene segmenta 
// ako je segment izvediv vraća se njegova cijena, u suprotnom se vraća vrijednost INF
double cost(int x, int y, int ei, int ej, vector<vector<pair<int, int> > > *cm, const vector<vector<short> >& map, vector<pair<vector<int>, double> > *g)
{
    double rez = 0.0;
    int m, n, mapWidth, mapHeight;
    double h = 0;
    
    mapWidth = map.size();
    
    if(mapWidth > 0)
        mapHeight = map[0].size();
    else return INF;


    // ***********************************************************************
    // ************* PROVJERA KOLIZIJE IZMEĐU VOZILA I PREPREKA  *************
    // ***********************************************************************
        
    for (unsigned int i = 0; i < cm[ei][ej].size(); i++)
    {
      m = node_dist * x + cm[ei][ej][i].first;
      n = node_dist * y + cm[ei][ej][i].second;
      
      // ako se bilo koja od čelija koju vozilo okupira tijekom izvođenja segmenta putanje nalazi 
      // izvan granica karte, vraća se vrijednost 100000  
      if (m < 0 || n < 0 || m > (mapWidth - 1) || n > (mapHeight - 1))
      {
        return INF;
      }
      else 
      {
        if (map[m][n] < 0)  // ako čelija zahvaća prepreku vraća se vrijednost 100000
          return INF;
        else
          rez += map[m][n];
      }
    }
    // **********************************************************************
    
    int vx = x + g[ei][ej].first[0];
    int vy = y + g[ei][ej].first[1];
        
    int dx = xf - vx;
    int dy = yf - vy;
        
    h = sqrt(dx * dx + dy * dy);    // heuristična komponenta cijene
    //fprintf(stderr, "xf: %d yf: %d vx: %d vy: %d h: %f g: %f\n", xf, yf, vx, vy, h, g[ei][ej].second);

    return (g[ei][ej].second + h);  // ako sve čelije segmenta putanje leže u slobodnom prostoru karte, vraća se cijena segmenta
}

// graph search algoritm (Dijkstra + A*)
void graphSearch(matvar_t * varVars, matvar_t * varCost, const vector<vector<short> >& vMap, int start[], int finish[], vector<int> wrX, vector<int> wrY, double** M, int sizeM[], int** veh_orientations)
{
    // varVars -> sadrži informacije o latici; za svaku početnu orijentaciju sadrži koord. susjednih čvorova, konačne orijentacije i cijene pripadajućih segmenta
    // varCost -> koordinate svih čelija kroz koje vozilo prolazi prilikom gibanja pojedinim segmentom u latici
    
    
    vector<int> wrongx;
    vector<int> wrongy;

    // početne koordinate i orijentacija vozila
    int xs =  start[0];
    int ys =  start[1];
    int fis = start[2];
    
    xf = finish[0];
    yf = finish[1];
    fif = finish[2];
    
    //publishanje posjećenih čvorova (za prikaz u RViz-u)
    geometry_msgs::Point a;
    nav_msgs::GridCells gCells;
    gCells.header.frame_id = "/Alfa/map";
    gCells.cell_width = mapResolution;
    gCells.cell_height = mapResolution;
    gCells.cells.clear();

    map<Vector, int> hm;
    map<Vector, pair<int, int> > hm_sol;

    int m, n, mc, nc;
    double *data_in, *data_out, *cell_in, *map_in;

    vector<pair<vector<int>, double> > graph[16];
    vector<vector<pair<int, int> > > cost_map[16];
    
    vector<int> tmp;
    vector<pair<int, int> > vptmp;
    double sf;

    m = varVars->dims[0];
    n = varVars->dims[1];

    data_in = (double*)varVars->data;

    // petlja po svim segmentima u latici, počevši od segmenata s početnom orijentacijom 0  
    for (int i = 0; i < m; i++)
    {
        tmp.clear();
        tmp.push_back((int)data_in[m + i]);     // x - koordinata susjednog čvora
        tmp.push_back((int)data_in[2*m + i]);   // y - koordinata susjednog čvora
        tmp.push_back((int)data_in[3*m + i]);   // konačna orijentacija u susjednom čvoru 
        sf = data_in[4*m + i];                  // cijena (duljina pripadajućeg segmenta)
            
        // indexi elemenata vektora "graph" predstavljaju početne orijentacije (0-15)
        // svaki element vektora "graph" sadrži informacije o svim 
        // susjednim čvorovima u koje se može doći s dotičnom početnom orijentacijom
        // informacije o susjednim čvorovima sadržane su u parovima oblika: 
        // (x-y koordinate susjednog čvora + konačna orijentacija, cijena pripadajućeg segmenta) 
        graph[(int)data_in[i]].push_back(make_pair(tmp, sf));   

        matvar_t* matvar = Mat_VarGetCell(varCost, i);
        mc = matvar->dims[0];
        nc = matvar->dims[1];
        cell_in = (double*)matvar->data;

        vptmp.clear();
        for (int j = 0; j < nc; j++)
        {
            vptmp.push_back(make_pair((int)cell_in[2*j], (int)cell_in[2*j+1]));
        }
            
        // indexi elemenata vektora "cost_map" predstavljaju početne orijentacije (0-15)
        // svaki element vektora "cost_map" sadrži vektore parova s
        // x-y koordinatama svih čelija kroz koje vozilo prolazi prilikom 
        // izvođenja pojedinog segmenta s dotičnom početnom orijentacijom
        cost_map[(int)data_in[i]].push_back(vptmp);
    }

    // definicija vektora za pohranu posjećenih čvorova prilikom pretraživanja grafa
    // koristi se kao sortirano binarno stablo (min-heap)
    // korijen stabla (prvi element vektora) je čvor s najmanjom cijenom
    vector<pair<Vector, double> > heap;
    
    double c;
    double new_c;
    //double pen = 0.0; // penalizacija promjena kuta
    bool path_exist = true;

    Vector v;
    Vector new_v;
        
    // spremanje početnih koordinata i orijentacije vozila u vektor "v"
    v.push_back(xs);
    v.push_back(ys);
    v.push_back(fis);

    heap.push_back(make_pair(v, 0.0));
    
    bool found;

    while(true)
    {
        if (heap.size() == 0)
        {
            path_exist = false;
            break;
        }
        swap(0, heap.size() - 1, &heap, &hm);   // zamjena mjesta prvog i posljednjeg čvora u stablu -> čvor s najmanjom cijenom dolazi na dno stabla

        // pomicanje u sljedeći čvor s najmanjom cijenom iz kojega će se dalje provjeravati ukupne cijene dolaska u njegove susjedne čvorove 
        v = heap.back().first;  // koordinate trenutnog čvora (x,y,fi)
        c = heap.back().second; // ukupna minimalna cijena dolaska u trenutni čvor iz početnog čvora
        hm[v] = -1;

        heap.pop_back();    // uklanjanje čvora s najmanjom cijenom iz binarnog stabla (to je trenutno zadnji čvor u heap-u)
        bubble_down(0, &heap, &hm); // spuštanje elementa koji se privremeno našao na root mjestu binarnog stabla (nakon prethodne swap operacije)
        
        // publishanje posjećenih čvorova (za prikaz u RViz-u)
        /*a.x = v[0] * node_dist * mapResolution;
        a.y = v[1] * node_dist * mapResolution;
        a.z = 0.0;
        gCells.cells.push_back(a);
        gCells.header.stamp = ros::Time::now();
        publishVisitedNodes(gCells);*/

        // određivanje ciljnih koordinata kod removinga
        if (v[0] == xf  &&  v[1] == yf  &&  v[2] == fif) // ako su koord. tenutnog čvora jednake koordinatama ciljnog čvora..
        {
            //fprintf(stderr, "Pronađen ciljni čvor!!\n");
            break;  // prekida se proces planiranja putanje..
        }

        // određivanje najkraćih putanja do svih susjednih čvorova pod pretpostavkom da se vozilo kreće unaprijed
        for (unsigned int i = 0; i < graph[v[2]].size(); i++)   // graph[v[2]] sadrži podatke o svim susjednim čvorovima u koje se može doći iz trenutnog čvora "v" uz početnu orijentaciju v[2] 
        {
            // u "new_v" se upisuju x-y koordinate i konačne orijentacije i-tog susjednog čvora koji je povezan s čvorom "v",
            // a u koji se iz čvora "v" može doći uz početnu orijentaciju v[2]
            new_v.clear();
            new_v.push_back(v[0] + graph[v[2]][i].first[0]);    // x - koordinata i-tog susjednog čvora
            new_v.push_back(v[1] + graph[v[2]][i].first[1]);    // y - koordinata i-tog susjednog čvora
            new_v.push_back(graph[v[2]][i].first[2]);           // konačna orijentacija u i-tom susjednom čvoru

            if (hm.find(new_v) == hm.end()) // ako "new_v" još nije dodan u mapu "hm"..
            {
                //heap.push_back(make_pair(new_v, c + graph[v[2]][i].second + pen * abs(new_v[2] - v[2])));
                new_c = cost(v[0], v[1], v[2], i, cost_map, vMap, graph);   // provjera izvedivosti i određivanje cijene segmenta
                        
                if (new_c < INF)    // ako je segment izvediv
                {
                    heap.push_back(make_pair(new_v, c + new_c));    // čvor se dodaje na kraj stabla zajedno s pripadajućom ukupnom cijenom
                    hm[new_v] = heap.size() - 1;        // pamti se index čvora u stablu
                    hm_sol[new_v] = make_pair(v[2], i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"
                    bubble_up(heap.size() - 1, &heap, &hm); // čvor se podiže na odgovarajuće mjesto u stablu tako da stablo bude sortirano 
                }
            } 
            else 
            {
                if (hm[new_v] != -1)    
                {
                    // ako je cijena dolaska u susjedni čvor "new_v" manja od prehodne cijene za taj čvor..
                    if (c + cost(v[0], v[1], v[2], i, cost_map, vMap, graph) < heap[hm[new_v]].second)
                    {
                        if(heap[hm[new_v]].first[0] != new_v[0] || heap[hm[new_v]].first[1] != new_v[1] || heap[hm[new_v]].first[2] != new_v[2])
                            fprintf(stderr, "Error in graph search!\n");

                        //heap[hm[new_v]].second = c + graph[v[2]][i].second + pen * (abs(new_v[2] - v[2]));
                        heap[hm[new_v]].second = c + cost(v[0], v[1], v[2], i, cost_map, vMap, graph); // upisuje se nova cijena do čvora "new_v"
                        hm_sol[new_v] = make_pair(v[2], i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"  
                        bubble_up(hm[new_v], &heap, &hm);   // modificira se mjesto "new_v" čvora u stablu tako da stablo ponovno bude sortirano
                    }
                }
            }
        }

        // određivanje najkraćih putanja do svih čvorova pod pretpostavkom da se vozilo kreće unatrag
        for (unsigned int i = 0; i < graph[(v[2] + 8) % 16].size(); i++)    // graph[(v[2] + 8) % 16] sadrži podatke o svim susjednim čvorovima u koje se može doći iz trenutnog čvora "v" kretanjem unatrag 
        {
            // u "new_v" se upisuju x-y koordinate i konačne orijentacije i-tog susjednog čvora koji je povezan s čvorom "v",
            // a u koji se iz čvora "v" može doći uz početnu orijentaciju (v[2] + 8) % 16 (vožnjom unatrag)
            new_v.clear();
            new_v.push_back(v[0] + graph[(v[2] + 8) % 16][i].first[0]);     // x - koordinata i-tog susjednog čvora
            new_v.push_back(v[1] + graph[(v[2] + 8) % 16][i].first[1]);     // y - koordinata i-tog susjednog čvora
            new_v.push_back((graph[(v[2] + 8) % 16][i].first[2] + 8) % 16); // konačna orijentacija u i-tom susjednom čvoru

            if (hm.find(new_v) == hm.end()) // ako "new_v" još nije dodan u mapu "hm"..
            {
                //heap.push_back(make_pair(new_v, c + graph[v[2]][i].second + pen * abs(new_v[2] - v[2])));
                new_c = 2 * cost(v[0], v[1], (v[2] + 8) % 16, i, cost_map, vMap, graph);    // provjera izvedivosti i određivanje cijene segmenta
                if (new_c < INF)    // ako je segment izvediv
                {
                    heap.push_back(make_pair(new_v, c + new_c));    // čvor se dodaje na kraj stabla zajedno s pripadajućom ukupnom cijenom
                    hm[new_v] = heap.size() - 1;    // pamti se index čvora u stablu
                    hm_sol[new_v] = make_pair((v[2] + 8) % 16 + 16, i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"
                    bubble_up(heap.size() - 1, &heap, &hm); // čvor se podiže na odgovarajuće mjesto u stablu tako da stablo bude sortirano
                }
            } 
            else 
            {
                if (hm[new_v] != -1)
                {
                    // ako je cijena dolaska u susjedni čvor "new_v" manja od prehodne cijene za taj čvor..
                    if (c + 2*cost(v[0], v[1], (v[2] + 8) % 16, i, cost_map, vMap, graph) < heap[hm[new_v]].second)
                    {
                        if(heap[hm[new_v]].first[0] != new_v[0] ||  heap[hm[new_v]].first[1] != new_v[1] || heap[hm[new_v]].first[2] != new_v[2])
                            fprintf(stderr, "Error in graph search!\n");

                        //heap[hm[new_v]].second = c + graph[v[2]][i].second + pen * (abs(new_v[2] - v[2]));
                        heap[hm[new_v]].second = c + 2*cost(v[0], v[1], (v[2] + 8)%16, i, cost_map, vMap, graph); // upisuje se nova cijena do čvora "new_v"
                        hm_sol[new_v] = make_pair((v[2] + 8) % 16 + 16, i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"
                        bubble_up(hm[new_v], &heap, &hm);   // modificira se mjesto "new_v" čvora u stablu tako da stablo ponovno bude sortirano
                    }
                }
            }
        }
    }

    if (path_exist) // ako je pronađena izvediva putanja
    {
        vector<pair<int, int> > sol;
        pair<int, int> p;
        vector<int> orient;
            
        v[0] = xf; v[1] = yf; v[2] = fif;
            
        // kretanje unatrag od ciljnog čvora do početnog čvora
        while (true)
        {
            // kada se dođe do početnog čvora prekida se while petlja
            if (v[0] == xs  &&  v[1] == ys  &&  v[2] == fis)
            {
                break;
            }

            p = hm_sol[v];
            sol.push_back(p);   // pohrana početne orijentacije u prethodnom čvoru i indexa segmenta putanje koji povezuje prethodni čvor s trenutnim čvorom "v" 

            // pomicanje u prethodni čvor (tj. u jedan čvor bliže početnom čvoru) 
            v[0] = v[0] - graph[p.first % 16][p.second].first[0];
            v[1] = v[1] - graph[p.first % 16][p.second].first[1];

            if (p.first > 15)
            {
                v[2] = (p.first + 8) % 16;
            }
            else
                v[2] = p.first % 16;
                
            orient.push_back(v[2]);
        }
        
        // vektor "veh_orientations" sadrži stvarne orijentacije vozila na početku svakog segmenta, počevši od zadnjeg prema prvom
        *veh_orientations = new int[orient.size() * sizeof(int)];
        for (int i = orient.size() - 1; i >= 0; i--)
        {
            (*veh_orientations)[i] = orient[i];
        }
        
        *M = new double [2 * sol.size() * sizeof(double)];
        //*M = new double [2 * 336 * sizeof(double)];
        data_out = *M;
        sizeM[0] = 2;
        sizeM[1] = sol.size();  // ukupan broj segmenata od kojih se sastoji izračunata putanja
        //sizeM[1] = 336;   // ukupan broj segmenata od kojih se sastoji izračunata putanja

        int k = 0;
        for (int i = sol.size() - 1; i >= 0; i--)
        {
            data_out[k] = sol[i].first % 16;    // početna orijentacija u i-tom čvoru
            data_out[k+1] = sol[i].second;      // index segmenta koji uz danu početnu orijentaciju vodi do sljedećeg čvora na putanji
            k += 2;
        }
        
        // ZA ISCRTAVANJE CIJELE LATICE:
        /*for(int j = 0; j < 16; j++)
        {
            for (unsigned int i = 0; i < graph[j].size(); i++)
            {
                data_out[k] = j;        // početna orijentacija u i-tom čvoru
                data_out[k+1] = i;      // index segmenta koji uz danu početnu orijentaciju vodi do sljedećeg čvora na putanji
                k += 2;
            }
        }*/
    }
    else
    {
        *M = NULL;
        data_out = *M;
        sizeM[0] = 0;
        sizeM[1] = 0;
    }
}

// **************************************************************************************
