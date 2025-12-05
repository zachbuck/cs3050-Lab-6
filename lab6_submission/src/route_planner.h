
#define MAX_VERTICES 10000
#define MAX_EDGES 50000
#define EARTH_RADIUS 6371.0

/*
	Generic Data structures needed for dynamic data allocation
*/

typedef struct _Node {
	void* data;

	struct _Node* next;
	struct _Node* prev;
} Node;

typedef struct {
	Node* first;
	Node* last;
} LinkedList;

typedef struct _PriorityNode {
	void* data;
	double priority;

	struct _PriorityNode* next;
} PriorityNode;

typedef struct {
	PriorityNode* first;
} PriorityQueue;

/*
	functions on the generic data structures
*/

Node* create_node(void*);
void* free_node(Node*);

LinkedList* create_linked_list();
void push_linked_list(LinkedList*, void*);
void free_linked_list(LinkedList*, int);

PriorityNode* create_priority_node(void*, double);
void* free_priority_node(PriorityNode*);

PriorityQueue* create_priority_queue();
void push_priority_queue(PriorityQueue*, void*, double);
void* pull_priority_queue(PriorityQueue*);
int priority_queue_is_empty(PriorityQueue*);
void free_priority_queue(PriorityQueue*);

/*
	Data structures needed for the graph operations
*/

typedef struct {
	void* vertex; // always Vertex*
	double weight;
} Connection;

typedef struct {
	int id;
	int index;

	double lat;
	double lon;

	int earliest;
	int latest;	

	LinkedList* connections; // contains Connection*
} Vertex;

typedef struct {
	int from;
	int to;
	double weight;
} Edge;

typedef struct {
	int vertex_count;
	Vertex* vertices[MAX_VERTICES];

	int edge_count;
	Edge* edges[MAX_EDGES];
} Graph;

typedef struct {
	int nodes_visited;
	double distance;

	LinkedList* vertices; // contains Vertex* on success, contains char* on error
} Path;

/*
	functions on the graph data structures
*/

Graph* create_graph();
void add_vertex(Graph*, Vertex*);
void add_edge(Graph*, Edge*);
void free_graph(Graph*);

Vertex* create_vertex(int, double, double, int, int);
void free_vertex(Vertex*);

Edge* create_edge();
void free_edge(Edge*);

Connection* create_connection(Vertex*, double);
// connections are freed in free_linked_list so no seperate free is required

Path* create_path(int, double);
void push_path(Path*, Vertex*);
void free_path(Path*);
void print_path(Path*);
void print_path_error(Path*);
int path_is_error(Path*);

Path* dijkstra(Graph*, Vertex*, Vertex*);
