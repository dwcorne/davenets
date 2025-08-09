/****************************************************************************************************\
  DAVENETS

  Discrete Activity on Very Edifying NETworks

  This is for simulating the activity of agents as they move on a structure that is defined
  by nodes and links. E.g. buses moving along their routes, picking up passengers and
  dropping them off.

\****************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#define PERSON_DEM 0
#define PARCEL_DEM 1

#define PERSON_LOAD_SECONDS 10
#define PARCEL_LOAD_SECONDS 6

#define PERSON_UNLOAD_SECONDS 5
#define PARCEL_UNLOAD_SECONDS 20

#define BUFFSIZE 100
#define MAXID 50
#define MAXGENDER 10
#define MAXSTOPS 100

#define ORDINARY 0
#define BUSSTOP 1
#define LOCKER 2
#define CITY 3
#define TOWN 4
#define RURAL 5
#define VILLAGE 6
#define SHOPPING 7



#define PENDING 0
#define ACTIVE 1
#define DONE 2
#define TRAVEL 3
#define LOADING 4 // or unloading
#define UNTOUCHED 5
#define INTRANSIT 6
#define DELIVERED 7
#define WAITING 8 // parcel or person waiting at a stop to start their travel

#define BUS_AGENT 0
#define OTHER_AGENT1 1
#define OTHER_AGENT2 2
#define PERSON_AGENT 3
#define PARCEL_AGENT 4

int TIMESTEP_SECONDS = 1;

typedef struct node {
  int id, index;
  double lat, lon;
  char type; // 0 ordinary // 1 bus stop  // 2 locker stop 
  char active;
} NODE;
NODE *nodes;

int global_nnodes = 0;

typedef struct link {
  int id;
  NODE *a, *b;
  int type;
  char active;
  double speed_metres_per_second, length_metres;
} LINK;
LINK *links;
int global_nlinks = 0;

typedef struct bus {
  int id, route_id, bus_id;
  int unassisted_capacity, wheelchair_capacity, wheelchair_equivalent1, wheelchair_equivalent2, parcel_number_capacity;
  double parcel_volume_capacity_cubic_metres, parcel_weight_capacity_kg, parcel_volume_payload_cubic_metres, parcel_weight_payload_kg;

  int unassisted_payload, wheelchair_payload, parcel_payload;

  int *times; // the route at route_id has nstops - these are the absolute times (in seconds) it is meant to leave those stops
  int nstops;

  int at_stop_seconds;

} BUS;

typedef struct route {

  int npoints;
  int *link_indices; // the route moves across these linkids  making a stop at each along. If along is -1 it doesn't stop there
  double *alongs;
  
} ROUTE;

typedef struct bus_route {
  char idstr[MAXID];
  int nstops;
  NODE *stop_nodes[MAXSTOPS];
  ROUTE r;
  int nbuses;
  BUS *buses;
} BUS_ROUTE;

BUS_ROUTE *bus_routes;
int global_nbusroutes = 0;

typedef struct agent {
  int id;
  int status; // PENDING  ACTIVE DONE
  char type;  // BUS
  char gender[MAXGENDER]; // m, f, etc...
  
  int id1, id2, id3;  // if bus, these are route_id and bus_id
                      // if person these are: node index where they gt on (id1) and off (id2), and time(seconds) (id3)  they get to id1 node

  double weight, volume;
  // state info, if active

  LINK *link_position;
  double link_along;
  int route_index;
  
  int operation_status;
  int restart_travel; // if operation_status is not travel, this is the time when we will restart travel
  int at_stop_id, at_stop_bus_route_index;

  int started, finished;
  // for buses
   
} AGENT;

AGENT *agents;
int nagents = 0;

// Structure to represent a 2D point
typedef struct {
    double x;
    double y;
} Point;


FILE *robustpickup(char *msg, char *fname);
int pickup_network(char *fname);
int pickup_busroutes(char *fname);
int getnode(FILE *f);
int getlink(FILE *f);
NODE *addnode(int id, double lat, double lon, char *type);
LINK *addlink(int id, int aid, int bid, double len, double sp, char *type);
NODE *getnodeptr(int node_id);
BUS_ROUTE *new_busroute(char *idstr);
int add_route_node(BUS_ROUTE *b, int nid);
int getroute(FILE *f);
int getbus(FILE *f);
int show_all(void);
int getrouteindex(char *name);
int HHMMtoseconds(char *t);
int HHMMSStoseconds(char *t);
int get_bus_parameters(FILE *f, BUS *b);
int get_bus_times(FILE *f, BUS *b, int n);
int build_agents(void);
AGENT *new_agent(void);
AGENT *create_bus_agent(BUS *b);
AGENT *create_person_agent(int s, NODE *a, NODE *b, char *gender);
int getearliestagentstart(void);
int run_simulation(void);
int agents_not_done(void);

int determine_routes(void);
int determine_route(BUS_ROUTE *r);
int getlinkalong(NODE *n, int *lindex, double *along);
double point_segment_distance(Point point, Point segment_start, Point segment_end, double *along);
int get_bus_start_position(AGENT *a, BUS_ROUTE *br);
int get_new_position(double a_previous, double sp, double len, double *along_new);
int just_reached_stop(AGENT *a, BUS_ROUTE *br, BUS *b, double sp);
int get_latlon(AGENT *a, double *ll);
double toRadians(double degree);
double lldistance(double lat1, double lon1, double lat2, double lon2);
int pickup_demand(char *fname);
int gettrip(FILE *f);
AGENT *create_parcel_agent(int s, NODE *a, NODE *b, double w, double v);
int initialize_stop(AGENT *a, BUS_ROUTE *br, BUS *b, int si, int t); // WRH define this
int get_people(FILE *f, NODE *a, NODE *b);
int get_parcels(FILE *f, NODE *a, NODE *b);
int create_demand_person(char *g, char *t, NODE *a, NODE *b);
int create_demand_parcels(char *t, NODE *a, NODE *b, double pw, double pv);
int setup_bus(AGENT *a, BUS *b);

int fits(AGENT *thing, BUS *b);
int install(AGENT *thing, BUS *b, int t);

int fits_person(AGENT *p, BUS *b);
int fits_parcel(AGENT *p, BUS *b);
int validate_gender(char *gender);

int unload_parcel(AGENT *p, BUS *b, int t);
int unload_person(AGENT *p, BUS *b, int t);

int round_up(int t);
double pcints(int a, int b);
int add_dem(int fromid, int toid, int t, float w, float v, int what);

typedef struct demrec {

  int fid, tid;
  int n;
  int type;
  int *t;
  char *g;
  float weight, volume; // kg and cubic m
} DEMREC;

DEMREC *demrecs;
int ndemrecs = 0;
int newdemrec(int fid, int tid, int type);
int add_person_to_dem(DEMREC *d,int arr,char g);

int output(void);
int gen_demand(int num);
int check_stop_arrivals(AGENT *a, BUS_ROUTE *r, BUS *b, int stop_id, int t);
int gend(int fromtype, int totype);

int gen_parcel_demand(int argc, char **argv);
int main(int argc, char **argv)
{

  srand((unsigned)clock());


  pickup_network(argv[1]);

  pickup_busroutes(argv[2]);  

  gen_demand(atoi(argv[3]));

  gen_parcel_demand(argc, argv);  

  output();
}


int output(void)
{
  int i, j, h, m, s, secs, secsover, pers_count=0, parc_count=0;
  DEMREC *d;
  char what[10];

  // do counts
    
  
  ///  printf("THAT's %d people  and %d parcels\n", pers_count, parc_count);
  
  for(i=0;i<ndemrecs;i++)
    {
      d = &demrecs[i];

      strcpy(what,"people");
      if(d->type==PARCEL_DEM)       strcpy(what,"parcels");
	
      printf("trip\n%d %d\n%s\n", d->fid, d->tid,what);
      for(j=0;j<d->n;j++)
	{
	  if(d->type==PARCEL_DEM)  
	    printf(" 01:00  %.2f %.2f ", d->weight, d->volume);
	  else
	    {
	      secs = d->t[j];
	      h = secs/3600;
	      secsover = secs - h*3600;
	      m = secsover/60;
	      s = secsover%60;
	      
	      printf(" %c %02d:%02d:%02d ", d->g[j], h, m, s);
	    }
	}
      printf("\nend%s\nendtrip\n\n",what);
    }

}


int gen_demand(int num)
{

  // initialize demand generation
  // place 1000 people at each city, 200 at each town 30 at each village

  int sofar = 0;
  while(sofar < num)
    {
      int r = rand()%100;

      if(r<=30) {sofar += gend(CITY,CITY);}
      else       if(r<=40) {sofar += gend(CITY,TOWN);}
      else       if(r<=45) {sofar += gend(CITY,SHOPPING);}
      else       if(r<=54) {sofar += gend(CITY,VILLAGE);}
      else       if(r<=58) {sofar += gend(CITY, RURAL);}
      else       if(r<=68) {sofar += gend(VILLAGE,VILLAGE);}
      else       if(r<=75) {sofar += gend(VILLAGE,SHOPPING);}
      else       if(r<=85) {sofar += gend(VILLAGE,TOWN);}
      else       if(r<=90) {sofar += gend(VILLAGE,RURAL);}      
      else       if(r<=95) {sofar += gend(SHOPPING,RURAL);}
      else       if(r<=100) {sofar += gend(RURAL,RURAL);}   					 
    						   
    }
}

int gen_parcel_demand(int argc, char **argv)
{

  int i, j, n, f, t;
  float w, v;
  
  for(i=4;i<argc;i+=3)
    {
      n = atoi(argv[i]);
      f = atoi(argv[i+1]); 
      t = atoi(argv[i+2]);     
      w = 3;
      v = 0.01;
      for(j=0;j<n;j++)
	add_dem(f, t, 3600, w, v, PARCEL_DEM);
    }

}


int gend(int fromtype, int totype)
{

  // get starting point
  int i, bri, bi, fromstop, tostop;

  //choose a route at random

  if(global_nbusroutes==0)
    {
      printf("No bus routes (or no correctly configured bus routes) are available, so I can't generate demand for them.\n");
      exit(1);
    }
  
  bri = rand()%global_nbusroutes;

  BUS_ROUTE *brp = &bus_routes[bri];
  // and a bus

  if(brp->nbuses==0)
    {
      return 0;
    }  

  bi = rand()%brp->nbuses;

  BUS *bp = &brp->buses[bi];

  // choose a random stop node of the correct type
  int righttype[1000], n = 0;
  for(i=0;i<brp->nstops;i++)
    if(brp->stop_nodes[i]->type == fromtype) {righttype[n++] = i;}
  if(n>0)
    { fromstop = righttype[rand()%n];} // fromstop is index into stop_nodes }
  else return 0;
  
  // choose a random stop node of the correct type
  n=0;
  for(i=fromstop+1;i<brp->nstops;i++)
    if(brp->stop_nodes[i]->type == totype) {righttype[n++] = i;}  

  if(n>0)
    { tostop = righttype[rand()%n];} // fromstop is index into stop_nodes }
  else return 0;  

  add_dem(brp->stop_nodes[fromstop]->id, brp->stop_nodes[tostop]->id, bp->times[fromstop], 75.0, 0.5, PERSON_DEM);
  return 1;
  
}


int add_dem(int fromid, int toid, int t, float w, float v, int what)
{
  int i, found = -1;
  DEMREC *d;
  
  for(i=0;i<ndemrecs;i++)
    {
      d = &demrecs[i];
      if((d->fid==fromid) && (d->tid == toid) && (d->type == what)) {found = i; break;}
    }
  if(found < 0)
    {
      newdemrec(fromid, toid, what);
      found = ndemrecs - 1;
    }

  d = &demrecs[found];
  d->weight = w;
  d->volume = v;
  // arrive between 10 mins before and 30s late

  int arr = (t - 600) + rand()%630;
  char g;
  if(rand()%100<50) {g = 'm';} else {g = 'f';}

  add_person_to_dem(d,arr,g);
}

int add_person_to_dem(DEMREC *d,int arr,char g)
{
  d->n++;

  if(d->n == 1)
    {
      d->t = (int *)malloc(sizeof(int));
      d->g = (char *)malloc(sizeof(char));      
  }
  else
    {
      d->t = (int *)realloc(d->t, d->n * sizeof(int));
      d->g = (char *)realloc(d->g, d->n * sizeof(char));      
    }

  d->t[d->n-1] = arr;
  d->g[d->n-1] = g;
  
}

int newdemrec(int fid, int tid, int type)
{

  ndemrecs++;
  if(ndemrecs==1) {demrecs = (DEMREC *)malloc(sizeof(DEMREC));}
  else {demrecs = (DEMREC *)realloc(demrecs,ndemrecs * sizeof(DEMREC));}

  DEMREC *d = &demrecs[ndemrecs-1];
  d->fid = fid;
  d->tid = tid;
  d->n = 0;
  d->type = type;
}


int determine_routes(void)
{
  int i;

  for(i=0;i<global_nbusroutes;i++)
    {
      determine_route(&bus_routes[i]);
    }
}


/****************************************************************************************************\
 the route is just a series of node IDs
 we now need to figure out the link positions of each of those nodes,
 and figure out the route in terms of waypoits 

\****************************************************************************************************/

int determine_route(BUS_ROUTE *br)
{

  ROUTE *r = &br->r;
  r->npoints = 0;

  int placenumber = 0;
  int lindex[1];
  double along[1];
  
  while(1)
    {
      NODE *n = br->stop_nodes[placenumber];
      getlinkalong(n,lindex,along);

      if(placenumber==0){      
	r->npoints++;
	if(r->npoints==1)
	  {r->link_indices = (int *)malloc(sizeof(int)); r->alongs = (double *)malloc(sizeof(double));}
	else 
	  {r->link_indices = (int *)realloc(r->link_indices, r->npoints * sizeof(int)); r->alongs = (double *)realloc(r->alongs, r->npoints * sizeof(double));}
	r->link_indices[r->npoints-1] = lindex[0];
	r->alongs[r->npoints-1] = along[0];

	placenumber++;	
	continue;
      }
      // if here. placenumber > 0;

      if(r->alongs[r->link_indices[r->npoints-1]] != lindex[0])
	{
          // fill in the shortest route between these two links -- TODO
          // but don't add final link ...
	}
      r->npoints++;
      r->link_indices = (int *)realloc(r->link_indices, r->npoints * sizeof(int)); r->alongs = (double *)realloc(r->alongs, r->npoints * sizeof(double));
      r->link_indices[r->npoints-1] = lindex[0];
      r->alongs[r->npoints-1] = along[0];
      placenumber++;
      if(placenumber >= br->nstops) break;
      continue;

    }
}

/****************************************************************************************************\

 If this node is the start of a link, return that link and 0
 If end of a link, return that link index and 1

 else, find closest link, return that link and its intersection point

\****************************************************************************************************/

int getlinkalong(NODE *n, int *lindex, double *along)
{
  int i;
  for(i=0;i<global_nlinks;i++)
    if(links[i].a->id == n->id)
      {lindex[0] = i; along[0] = 0; return 0;}
    else  if(links[i].b->id == n->id)
      {lindex[0] = i; along[0] = 1; return 0;}  
  
  // if here we need to find closest link
  Point a, b, c, *ap, *bp, *cp;
  double d, bestd, along2[1];
  int bli;
  
  ap = &a;
  bp = &b;
  cp = &c;

  cp->x = n->lat;
  cp->y = n->lon;
  for(i=0;i<global_nlinks;i++)
    {
      LINK *l = &links[i];
      ap->x = l->a->lat;
      ap->y = l->a->lon;
      bp->x = l->b->lat;
      bp->y = l->b->lon;      
      
      d = point_segment_distance(c,a,b,along2);
      if((i==0) || (d < bestd)) {bestd = d; bli = i;}
    }  
  *lindex = bli;
  *along = along2[0];
}



int getearliestagentstart(void)
{
  int i, started=0, thist, et = -1;
  
  for(i=0;i<nagents;i++)
    {
      AGENT *a = &agents[i];
      if(a->type == BUS_AGENT)
	{
	  BUS *b = &bus_routes[a->id1].buses[a->id2];
	  thist = b->times[0];
	  if(started==0)
	    {started=1; et = thist;}
	  else if(thist<et) {et = thist;}
	}
      else if((a->type == PARCEL_AGENT)|| (a->type == PERSON_AGENT))
	{
	  thist = a->id3;
	  if(started==0)
	    {started=1; et = thist;}
	  else if(thist<et) {et = thist;}
	}
    }
  return et;

}

int round_up(int t)
{
  int i;
  AGENT *a;

  int ns_people = 0, nf_people = 0, ns_parcel = 0, nf_parcel = 0, t_people = 0, t_parcel = 0, n_people = 0, n_parcels = 0;
  
  
  for(i=0;i<nagents;i++)
    {  
      a = &agents[i];
      if(a->type == PERSON_AGENT)
	{
	  printf("PERSON:  %d %d\n", a->started, a->finished);
	  n_people++;

	  if(a->started<0) {ns_people++;}
	  else if(a->finished<0) {nf_people++;}
	  else t_people += a->finished - a->started;
	}
      else       if(a->type == PARCEL_AGENT)
	{
	  printf("PARCEL:  %d %d\n", a->started, a->finished);
	  n_parcels++;
	  if(a->started<0) {ns_parcel++;}
	  else if(a->finished<0) {nf_parcel++;}
	  else t_parcel += a->finished - a->started;
	}
    }

  printf("%.1f%% of people completed their journneys, total time %d seconds\n", pcints(n_people - (nf_people + ns_people), n_people), t_people);
  printf("%.1f%% of parcels completed their journeys, total time %d seconds\n", pcints(n_parcels - (nf_parcel + ns_parcel), n_parcels), t_parcel);	 

}

double pcints(int a, int b)
{
  if(b==0) return 0;
  double da = (double)a;
  double db = (double)b;  
  
  return 100 * da/db;
}

int run_simulation(void)
{

  // find out when is sensible to start in simulated_time -
  // this will be when earliest bus leaves its depot

  int starttime = getearliestagentstart();
  int the_time = starttime;
  
  printf("simulation starting at %d\n", starttime);

  while(agents_not_done())
    {
      the_time += TIMESTEP_SECONDS;
    }
  round_up(the_time); // collect results and provide output
}


int fits(AGENT *thing, BUS *b)
{
  if(thing->type == PERSON_AGENT)
    return fits_person(thing, b);
  else   if(thing->type == PARCEL_AGENT)
    return fits_parcel(thing, b);    
}

int unload_parcel(AGENT *p, BUS *b, int t)
{
  b->parcel_payload--;
  b->parcel_volume_payload_cubic_metres -= p->volume;
  b->parcel_weight_payload_kg -= p->weight;


  p->finished = t;
  p->status = DONE;

  printf("T = %d:  unloading parcel from bus\n", t);
  return PARCEL_UNLOAD_SECONDS;
}

int unload_person(AGENT *p, BUS *b, int t)
{
  b->unassisted_payload--;
  p->finished = t;
  p->status = DONE;

  printf("T = %d:  unloading person from bus\n", t);  
  return PERSON_UNLOAD_SECONDS;
}

int install(AGENT *thing, BUS *b, int t)
{

  thing->operation_status = INTRANSIT;
  thing->at_stop_id = -1;
  thing->started = t;
  
  if(thing->type == PERSON_AGENT)
    {   
      b->unassisted_payload += 1;  
      printf("T = %d:  boarding person onto bus\n", t);  
      return PERSON_LOAD_SECONDS;
    }
  else   if(thing->type == PARCEL_AGENT)
    {   
      b->parcel_payload += 1;
      b->parcel_volume_payload_cubic_metres += thing->volume;
      b->parcel_weight_payload_kg += thing->weight;
      
      printf("T = %d:   putting parcel onto bus\n", t);  
      return PARCEL_LOAD_SECONDS;
    }    
  
}


int fits_person(AGENT *p, BUS *b)
{
  int spare = b->unassisted_capacity - b->unassisted_payload;
  if(spare>=1) return 1;
  return 0;
}

int fits_parcel(AGENT *p, BUS *b)
{
  int spare = b->parcel_number_capacity - b->parcel_payload;
  if(spare<1) return 0;
  double sparev = b->parcel_volume_capacity_cubic_metres - b->parcel_volume_payload_cubic_metres;
  if(sparev<p->volume) return 0;
  double sparew = b->parcel_weight_capacity_kg - b->parcel_weight_payload_kg;
  if(sparew<p->weight) return 0;  
			 
  return 1;
}

int just_reached_stop(AGENT *a, BUS_ROUTE *br, BUS *b, double sp)
{
  // already there

  // what it was
  int was_index = a->at_stop_bus_route_index, was_id = a->at_stop_id;

  double ll[2], secdist = sp * ( (double) TIMESTEP_SECONDS);
  // is nearest stop on route within that distance
  int i, si;

  get_latlon(a,ll);
  Point p;
  Point *pr = &p;
  pr->x = ll[0];
  pr->y = ll[1];

  double d = 100*secdist, bestd;
  int start = was_index + 1;
  for(i=start;i<br->nstops;i++) // stop 0 is the depot
    {
      NODE *n = br->stop_nodes[i];
      double d = lldistance(ll[0],ll[1],n->lat,n->lon);
      if((i==start)||(d<bestd)) {bestd = d; si = i;}
    }
  d = bestd*1000; // KM to metres

  int atstop = 0;

  printf("checking for atstop -- best distance is %f metres, while 1 second travel is %f metres\n", d, secdist);
  if(d <= secdist)
    {
      atstop = 1;
    }

  if((atstop) && (was_index != si)) // we are at a new stop
    return si;

  // otherwise no
  return -1;
  
}

int get_latlon(AGENT *a, double *ll)
{

  double alat = a->link_position->a->lat, alon = a->link_position->a->lon, blat = a->link_position->b->lat, blon = a->link_position->b->lon;
  ll[0] = alat + a->link_along * (blat - alat);
  ll[1] = alon + a->link_along * (blon - alon);  
}


int get_new_position(double a_previous, double sp, double len, double *along_new)
{
  double tss = (double)TIMESTEP_SECONDS;

  double dist = sp * tss;

  double place = a_previous * len;

  place += dist;

  if(place>=len) along_new[0] = 1;
  else along_new[0] = place/len;

}



int get_bus_start_position(AGENT *a, BUS_ROUTE *r)
{
  ROUTE *rp = &r->r;
  a->link_position = &links[rp->link_indices[0]];
  a->link_along = rp->alongs[0];
  a->route_index = 0;
}

int agents_not_done(void)
{
  int i;
  AGENT *a;

  for(i=0;i<nagents;i++)
    {
      a = &agents[i];
      if((a->type == BUS_AGENT) && (a->status != DONE)) return 1;
    }
  return 0; // all the bus agents must have finished;
}

AGENT *create_bus_agent(BUS *b)
{
  AGENT *a = new_agent(); 
  a->type = BUS_AGENT;
  a->id1 = b->route_id;
  a->id2 = b->bus_id;
  return a;
}

AGENT *new_agent(void)
{
  nagents++;
  if(nagents==1) {agents = (AGENT *)malloc(sizeof(AGENT));}
  else  {agents = (AGENT *)realloc(agents,nagents*sizeof(AGENT));}

  AGENT *a = &agents[nagents-1];
  a->id = nagents-1;
  a->status = PENDING;
  a->id1 = a->id2 = a->id3 = a->started = a->finished = -1;
  a->weight = a->volume = -1;

  return a;

}

int build_agents(void)
{
  int i, j;

  for(i=0;i<global_nbusroutes;i++)
    {
      BUS_ROUTE *br = &bus_routes[i];
      
      for(j=0;j<br->nbuses;j++)
	{
	  BUS *b = &br->buses[j];
	  AGENT *a = create_bus_agent(b);
	}

    }
}


int show_all(void)
{
  int i, j, k;

  printf("there are %d nodes\n", global_nnodes);

  for(i=0;i<global_nnodes;i++)
    {
      NODE *n = &nodes[i];
      printf("id %d  %f %f  %d\n", n->id, n->lat, n->lon, n->type);
    }


  printf("there are %d links\n", global_nlinks);

    for(i=0;i<global_nlinks;i++)
    {
      LINK *n = &links[i];
      printf("id %d nodes  %d %d  %f %f  %d\n", n->id, n->a->id, n->b->id, n->length_metres, n->speed_metres_per_second, n->type);
    }

  printf("there are %d busroutes\n", global_nbusroutes);

   for(i=0;i<global_nbusroutes;i++)
    {
      BUS_ROUTE *n = &bus_routes[i];
      printf("  route %s has %d stops and %d buses\n", n->idstr, n->nstops, n->nbuses);

      for(j=0;j<n->nstops;j++)
	{	
	  printf(" %d \n", n->stop_nodes[j]->id);
	}
      printf("\n");

      for(j=0;j<n->nbuses;j++)
	{
	  printf("  bus %d\n", j+1);
	  BUS *bp = &n->buses[j];
	  for(k=0;k<n->nstops;k++)
	    printf("%d ", bp->times[k]);
	  printf("\n");	  
	}
      printf("\n");
    }

  printf("there are %d agents\n", nagents);

  for(i=0;i<nagents;i++)
    {
      AGENT *a = &agents[i];
      printf(" type %d:  %d %d %d   %f %f\n", a->type,a->id1,a->id2,a->id3,a->weight,a->volume);

    }

}

int pickup_network(char *fname)
{
  FILE *f = robustpickup("network file", fname);

  char instr[BUFFSIZE];
  int i, r;

  while(1)
    {
      r = fscanf(f,"%s",instr);
      if(r<1)break;
      if(instr[0]=='n') {getnode(f);}
      else       if(instr[0]=='l') {getlink(f);}
      else {printf("When reading the network file %s, I was expecting either n or l, but I got %s\n", fname,instr); exit(1);}
    }
  fclose(f);
}



int pickup_demand(char *fname)
{
  FILE *f = robustpickup("demand file", fname);

  char instr[BUFFSIZE];
  int i, r;

  while(1)
    {
      r = fscanf(f,"%s",instr);
      if(r<1)break;
      if(!strcmp(instr,"trip")) {gettrip(f);}
      else {printf("When reading the demand file %s, I was expecting trip, but I got %s\n", fname,instr); exit(1);}
    }
  fclose(f);
}

int gettrip(FILE *f)
{
  char instr[BUFFSIZE];
  int i, r;

  fscanf(f,"%s",instr);
  NODE *a = getnodeptr(atoi(instr));
  fscanf(f,"%s",instr);
  NODE *b = getnodeptr(atoi(instr));  

  while(1)
    {
     r = fscanf(f,"%s",instr);
     if(r<1) break;
     if(!strcmp(instr,"endtrip")) break;
     if(!strcmp(instr,"people")) get_people(f,a,b);
     if(!strcmp(instr,"parcels")) get_parcels(f,a,b);     
    }

}

int get_people(FILE *f, NODE *a, NODE *b)
{
  char instr[BUFFSIZE], gender[MAXGENDER];
  int i, r;

  while(1)
    {
     r = fscanf(f,"%s",instr);
     if(r<1) break;
     
     if(!strcmp(instr,"endpeople")) break;
     strcpy(gender,instr);
     validate_gender(gender);
     fscanf(f,"%s",instr);     
     create_demand_person(gender,instr,a,b);
    }    
}

int validate_gender(char *gender)
{
  if(!strcmp(gender,"m")) return 1;
  else   if(!strcmp(gender,"f")) return 1;
  else if(!strcmp(gender,"nb")) return 1;

  printf("don't currently recognize %s as a gender in the demand file - please fix\n", gender);
  exit(1);
}

int get_parcels(FILE *f, NODE *a, NODE *b)
{
  char instr[BUFFSIZE];
  int i, r, np;
  double pw,pv;
  
  while(1)
    {
     r = fscanf(f,"%s",instr);
     if(r<1) break;
     if(!strcmp(instr,"endparcels")) break;

     fscanf(f,"%s",instr);     pw = atof(instr);
     fscanf(f,"%s",instr);     pv = atof(instr);        
     create_demand_parcels(instr,a,b,pw,pv);
    }    
}

int create_demand_person(char *gender,char *t, NODE *a, NODE *b)
{
  int s = HHMMSStoseconds(t);
  create_person_agent(s,a,b,gender); 
}

int create_demand_parcels(char *t, NODE *a, NODE *b, double pw, double pv)
{
  int s = HHMMSStoseconds(t);
  create_parcel_agent(s,a,b,pw,pv); 
}


AGENT *create_person_agent(int s, NODE *na, NODE *nb, char *gender)
{
  AGENT *a = new_agent(); 
  a->type = PERSON_AGENT;
  a->id1 = na->index;
  a->id2 = nb->index;
  a->id3 = s;
  a->operation_status = UNTOUCHED;
  a->at_stop_id = -1;
  a->at_stop_bus_route_index = -1;  

  strcpy(a->gender, gender);
  return a;
  
}

AGENT *create_parcel_agent(int s, NODE *na, NODE *nb, double w, double v)
{
  AGENT *a = new_agent(); 
  a->type = PARCEL_AGENT;
  a->id1 = na->index;
  a->id2 = nb->index;
  a->id3 = s;
  a->weight = w;
  a->volume = v;
  a->operation_status = UNTOUCHED;
  a->at_stop_id = -1;
  a->at_stop_bus_route_index = -1;  
  return a;
  
}



int pickup_busroutes(char *fname)
{
  FILE *f = robustpickup("busroute file", fname);

  char instr[BUFFSIZE];
  int i, r;

  while(1)
    {
      r = fscanf(f,"%s",instr);
      if(r<1)break;
      if(!strcmp(instr,"STARTROUTE"))
	{getroute(f);}
      else  if(!strcmp(instr,"STARTBUS"))
	{getbus(f);}
      else {printf("When reading the bus-routek file %s, I was expecting STARTROUTE, but I got %s\n", fname,instr); exit(1);}
    }
  fclose(f);
}


FILE *robustpickup(char *msg, char *fname)
{
  FILE *f = fopen(fname,"r");
  if(f==(FILE *)NULL)
    {
      printf("Was trying to open the %s - %s - but couldn't\n", msg, fname); exit(1);
    }
  return f;
}

int getroute(FILE *f)
{

  char instr[BUFFSIZE],name[MAXID];
  int i,r, id, no_routes=0, nnodes=0, *nodeids;
  double lat, lon;

  r = fscanf(f,"%s",instr);
  if(r<1) {no_routes=1;}
  else if(!strcmp(instr,"ENDROUTE"))    {no_routes=1;}

  if(no_routes) return 0;

  BUS_ROUTE *b = new_busroute(instr);
  // now it is a list of node ids
  
  while(1)
    {
      r = fscanf(f,"%s",instr);
      if(r<1) break;
      if(!strcmp(instr,"ENDROUTE")) break;

      add_route_node(b,atoi(instr));
    }
}

int getbus(FILE *f)
{

  char instr[BUFFSIZE],name[MAXID], routename[MAXID];
  int i,r, id, no_routes=0, nnodes=0, *nodeids, routeid;
  double lat, lon;

  r = fscanf(f,"%s",instr);
  if(r<1) {no_routes=1;}
  else if(!strcmp(instr,"ENDBUS"))    {no_routes=1;}

  if(no_routes) return 0;

  strcpy(routename,instr);
  // so we have a bus agent connected to routeid instr

  routeid = getrouteindex(routename);
  if(routeid<0)
    {
      printf("the bus routes file seems to be defining a bus on route %s, but this route has not been defined yet - it should be defined earlier in the file\n",routename);
      exit(1);

    }
  BUS_ROUTE *b = &bus_routes[routeid];

  b->nbuses++;
  if(b->nbuses==1) {b->buses = (BUS *)malloc(sizeof(BUS));}
  else  {b->buses = (BUS *)realloc(b->buses,b->nbuses * sizeof(BUS));}

  BUS *bp = &b->buses[b->nbuses-1];
  bp->route_id = routeid;
  bp->bus_id = b->nbuses-1;
  
  get_bus_parameters(f,bp);
  get_bus_times(f,bp,b->nstops);

   fscanf(f,"%s",instr);
   if(strcmp(instr,"ENDBUS")!=0)
     {
       printf("something wrong with bus definition - instead of ENDBUS I saw %s\n", instr);
       exit(1);
     }


  // gt endbus?
}

int get_bus_parameters(FILE *f, BUS *b)
{

  char instr[BUFFSIZE];

  fscanf(f,"%s",instr);  b->unassisted_capacity = atoi(instr);
  fscanf(f,"%s",instr);  b->wheelchair_capacity = atoi(instr);
  fscanf(f,"%s",instr);  b->wheelchair_equivalent1 = atoi(instr);
  fscanf(f,"%s",instr);  b->wheelchair_equivalent2 = atoi(instr);      
  fscanf(f,"%s",instr);  b->parcel_number_capacity = atoi(instr);

  fscanf(f,"%s",instr);  b->parcel_volume_capacity_cubic_metres = atof(instr);
  fscanf(f,"%s",instr);  b->parcel_weight_capacity_kg = atof(instr);          
}

int get_bus_times(FILE *f, BUS *b, int n)
{

  b->times = (int *)malloc(n*sizeof(int));
  b->nstops = n;
  
  int i;
  char instr[BUFFSIZE];
  
  for(i=0;i<n;i++)
    {
      fscanf(f,"%s",instr);
      b->times[i] =  HHMMtoseconds(instr);
    }
}

int HHMMtoseconds(char *t)
{

  char d[5];
  int h, m;

  d[0] = t[0]; d[1] = t[1]; d[2] = '\0';
  h = atoi(d);
  d[0] = t[3]; d[1] = t[4]; d[2] = '\0';  
  m = atoi(d);

  return 3600*h + 60*m;
}


int HHMMSStoseconds(char *t)
{

  if(strlen(t)<7) {return HHMMtoseconds(t);}
  char d[5];
  int h, m, s;

  d[0] = t[0]; d[1] = t[1]; d[2] = '\0';
  h = atoi(d);
  d[0] = t[3]; d[1] = t[4]; d[2] = '\0';  
  m = atoi(d);
  d[0] = t[6]; d[1] = t[7]; d[2] = '\0';  
  s = atoi(d);  

  return 3600*h + 60*m + s;
}




int getrouteindex(char *name)
{
  int i;
  
  for(i=0;i<global_nbusroutes;i++)
    {
      if(!strcmp(bus_routes[i].idstr,name)) return i;
    }
  return -1;
}

int add_route_node(BUS_ROUTE *b, int nid)
{
  b->nstops++;
  if(b->nstops>MAXSTOPS)
    {
      printf("sorry - a bus route can't have more than %d stops\n",MAXSTOPS); exit(1);
    }

  NODE *n = getnodeptr(nid);
  b->stop_nodes[b->nstops-1] = n;
}

BUS_ROUTE *new_busroute(char *idstr)
{

  global_nbusroutes++;
  if(global_nbusroutes==1) {bus_routes = (BUS_ROUTE *)malloc(sizeof(BUS_ROUTE));}
  else {bus_routes = (BUS_ROUTE *)realloc(bus_routes,global_nbusroutes * sizeof(BUS_ROUTE));}

  BUS_ROUTE *b = &bus_routes[global_nbusroutes-1];
  strcpy(b->idstr,idstr);
  b->nstops = 0;
  b->nbuses = 0;

  return b;
}


int getnode(FILE *f)
{

  char instr[BUFFSIZE];
  int i,r, id;
  double lat, lon;

  fscanf(f,"%s",instr);  id = atoi(instr);
  fscanf(f,"%s",instr);  lat = atof(instr);
  fscanf(f,"%s",instr);  lon = atof(instr);
  fscanf(f,"%s",instr);

  nodes = addnode(id,lat,lon,instr);

}


int getlink(FILE *f)
{

  char instr[BUFFSIZE];
  int i,r, id, aid,bid;
  double len,sp;

  fscanf(f,"%s",instr);  id = atoi(instr);
  fscanf(f,"%s",instr);  aid = atoi(instr);
  fscanf(f,"%s",instr);  bid = atoi(instr);
  fscanf(f,"%s",instr);  len = atof(instr);
  fscanf(f,"%s",instr);  sp = atof(instr);  
  fscanf(f,"%s",instr);  
  
  links = addlink(id,aid,bid,len,sp,instr);

}

NODE *addnode(int id, double lat, double lon, char *type)
{
  global_nnodes++;
  if(global_nnodes==1) {nodes = (NODE *)malloc(sizeof(NODE));}
  else  {nodes = (NODE *)realloc(nodes,global_nnodes*sizeof(NODE));}

  NODE *n = &nodes[global_nnodes-1];
  n->id = id;
  n->lat = lat;
  n->lon = lon;
  n->index = global_nnodes-1;
  
  if(!strncmp(type,"ord",3)) { n->type = ORDINARY;}
  else   if(!strncmp(type,"bus",3)) { n->type = BUSSTOP;}
  else   if(!strncmp(type,"locker",3)) { n->type = LOCKER;}
  else   if(!strncmp(type,"city",3)) { n->type = CITY;}
  else   if(!strncmp(type,"town",3)) { n->type = TOWN;}
  else   if(!strncmp(type,"village",3)) { n->type = VILLAGE;}
  else   if(!strncmp(type,"rural",3)) { n->type = RURAL;}
  else   if(!strncmp(type,"shopping",3)) { n->type = SHOPPING;}            
  // or else ... can be others, but they are used for demand generation
  n->active = 0; 
  return nodes;
}

LINK *addlink(int id, int aid, int bid, double len, double sp, char *type)
{
  global_nlinks++;
  if(global_nlinks==1) {links = (LINK *)malloc(sizeof(LINK));}
  else  {links = (LINK *)realloc(links,global_nlinks*sizeof(LINK));}

  LINK *l = &links[global_nlinks-1];
  l->id = id;
  l->a = getnodeptr(aid);
  l->b = getnodeptr(bid);  
  l->length_metres = len;
  l->speed_metres_per_second = sp;
  
  if(!strncmp(type,"ord",3)) { l->type = ORDINARY;}
  //  else   if(!strncmp(type,"bus",3)) { n->type = BUSSTOP;}
  //else   if(!strncmp(type,"locker",3)) { n->type = LOCKER;}  
  // or else ...
  l->active = 0; 
  return links;
}

NODE *getnodeptr(int node_id)
{
  int i;
  for(i=0;i<global_nnodes;i++)
    if(nodes[i].id==node_id) return &nodes[i];

  printf("could not find node with id: %d\n",node_id); exit(1);
}






// Function to calculate the distance between two points
double distance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// Function to calculate the dot product of two vectors
double dot_product(Point v1, Point v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

// Function to calculate the distance from a point to a line segment
double point_segment_distance(Point point, Point segment_start, Point segment_end, double *along) {
    Point segment_vector = {segment_end.x - segment_start.x, segment_end.y - segment_start.y};
    Point point_vector = {point.x - segment_start.x, point.y - segment_start.y};

    double segment_length_squared = dot_product(segment_vector, segment_vector);

    // If the segment is a point, return the distance to that point.
    if (segment_length_squared == 0.0) {
        return distance(point, segment_start);
    }

    // Calculate the projection of the point vector onto the segment vector.
    double t = dot_product(point_vector, segment_vector) / segment_length_squared;

    // If the projection is outside the segment, clamp it to the segment endpoints.
    if (t < 0.0) {
      along[0] = 0;
      return distance(point, segment_start);
    } else if (t > 1.0) {
      along[0] = 1;
      return distance(point, segment_end);
    }

    // Calculate the closest point on the segment.
    Point closest_point = {segment_start.x + t * segment_vector.x, segment_start.y + t * segment_vector.y};

    // Return the distance between the point and the closest point on the segment.

    along[0] = distance(closest_point,segment_start) / distance(segment_start,segment_end);
    return distance(point, closest_point);
}


// converting degrees to radians
double toRadians(double degree)
{
    // cmath library in C++ 
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

double lldistance(double lat1, double lon1, double lat2, double lon2)
{
    // Convert the latitudes 
    // and longitudes
    // from degree to radians.
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);
    
    // Haversine Formula
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;

    double ans = pow(sin(dlat / 2), 2) + 
                          cos(lat1) * cos(lat2) * 
                          pow(sin(dlon / 2), 2);

    ans = 2 * asin(sqrt(ans));

    // Radius of Earth in 
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    double R = 6371;

    // Calculate the result
    ans = ans * R;

    return ans;
}
