
#include <stdlib.h>
#include <stdio.h>

#include "route_planner.h"

/* create_node
 - allocates a new node structure
 - takes a void* data which is the data stored in the node
 - returns a pointer to the node or NULL on error
*/
Node* create_node(void* data) {
	Node* node = malloc(sizeof(Node));
	if (node == NULL) { return NULL; }

	node->data = data;
	node->next = NULL;
	node->prev = NULL;

	return node;
}

/* free_node
 - frees a node structure
 - takes a pointer to a node
 - returns the contained data or NULL on error
*/
void* free_node(Node* node) {
	if (node == NULL) { return NULL; }

	void* data = node->data;
	free(node);

	return data;
}

/* create_linked_list
 - allocates a new linked list structure
 - returns a pointer to the new linked list or NULL on error
*/
LinkedList* create_linked_list() {
	LinkedList* list = malloc(sizeof(LinkedList));
	if (list == NULL) { return NULL; }

	list->first = NULL;
	list->last = NULL;

	return list;
}

/* push_linked_list
 - pushes an item to the end of the linked list
 - takes a pointer to the linked list and a void* with the data to be stored
*/
void push_linked_list(LinkedList* list, void* data) {
	if (list == NULL) { return; }

	Node* node = create_node(data);
	if (node == NULL) { return; }

	if (list->first == NULL) {
		list->first = node;
		list->last = node;
	} else {
		list->last->next = node;
		node->prev = list->last;
		list->last = node;
	}
}

/* free_linked_list
 - frees a linked list structure and optionally all of the data which was stored inside
 - takes a pointer to the linked list
 - takes an integer which makes the function free all contained data on != 0
*/
void free_linked_list(LinkedList* list, int free_data) {
	if (list == NULL) { return; }

	Node* current = list->first;
	while (current != NULL) {
		Node* next = current->next;
		void* data = free_node(current);
		if (free_data) {
			free(data);
		}
		current = next;
	}

	free(list);
}

/* create_priority_node
 - allocates a priority node structure
 - takes a void* data for the data stored in the node and a double for the priority of the node
 - returns a pointer to the node or NULL on error
*/
PriorityNode* create_priority_node(void* data, double priority) {
	PriorityNode* node = malloc(sizeof(PriorityNode));
	if (node == NULL) { return NULL; }

	node->data = data;
	node->priority = priority;
	node->next = NULL;

	return node;
}

/* free_priority_node
 - frees a priority node structure
 - takes a priority node to be freed
 - returns the data stored inside or NULL on error
*/
void* free_priority_node(PriorityNode* node) {
	if (node == NULL) { return NULL; }

	void* data = node->data;
	free(node);
	
	return data;
}

/* create_priority_queue
 - allocates a new priority queue structure
 - returns a pointer to the new priority queue or NULL on error
*/
PriorityQueue* create_priority_queue() {
	PriorityQueue* queue = malloc(sizeof(PriorityQueue));
	if (queue == NULL) { return NULL; }

	queue->first = NULL;

	return queue;
}

/* push_priority_queue
 - pushes an item into the priority queue
 - takes a pointer to a priority queue, a piece of data to be stored, and the priority of that piece of data in the queue
*/
void push_priority_queue(PriorityQueue* queue, void* data, double priority) {
	if (queue == NULL) { return; }

	PriorityNode* node = create_priority_node(data, priority);
	if (node == NULL) { return; }

	if (queue->first == NULL) {
		queue->first = node;
	} else if (priority < queue->first->priority) {
		node->next = queue->first;
		queue->first = node;
	} else {
		PriorityNode* before = queue->first;
		PriorityNode* after = queue->first->next;
		while (after != NULL) {
			if (priority < after->priority) { break; }
			before = after;
			after = after->next;
		}

		before->next = node;
		node->next = after;
	}
}

/* pull_priority_queue
 - pulls the item with the lowest priority in the queue
 - takes a pointer to a priority queue
 - returns the data with the lowest priority or NULL on error or empty
*/
void* pull_priority_queue(PriorityQueue* queue) {
	if (queue == NULL) { return NULL; }
	if (queue->first == NULL) { return NULL; }

	PriorityNode* node = queue->first;
	queue->first = node->next;

	void* data = node->data;
	free_priority_node(node);

	return data;
}

/* priority_queue_is_empty
 - returns if a priority queue is empty
 - takes a pointer to a priority queue
 - returns 1 if the priority queue is empty or NULL and 0 if it is not
*/
int priority_queue_is_empty(PriorityQueue* queue) {
	if (queue == NULL) { return 1; }

	return queue->first == NULL ? 1 : 0;
}

/* free_priority_queue
 - frees a priority queue
 - takes a pointer to a priority queue
*/
void free_priority_queue(PriorityQueue* queue){ 
	if (queue == NULL) { return; }

	while (queue->first != NULL) {
		pull_priority_queue(queue);
	}

	free(queue);
}

/* create_graph
 - allocates a new graph structure
 - returns a pointer to the new graph structure or NULL on error
*/
Graph* create_graph() {
	Graph* graph = malloc(sizeof(Graph));
	if (graph == NULL) { return NULL; }

	graph->vertex_count = 0;
	graph->edge_count = 0;

	for (int i = 0; i < MAX_VERTICES; i++) {
		graph->vertices[i] = NULL;
	}

	for (int i = 0; i < MAX_EDGES; i++) {
		graph->edges[i] = NULL;
	}

	return graph;
}

/* add_vertex
 - adds a vertex to a graph
 - takes a graph and the vertex to be added to that graph
*/
void add_vertex(Graph* graph, Vertex* vertex) {
	if (graph == NULL) { return; }
	if (vertex == NULL) { return; }

	vertex->index = graph->vertex_count;
	graph->vertices[graph->vertex_count] = vertex;
	graph->vertex_count += 1;
}

/* add_edge
 - adds an edge to a graph
 - takes a graph and an edge to be added
*/
void add_edge(Graph* graph, Edge* edge) {
	if (graph == NULL) { return; }
	if (edge == NULL) { return; }

	graph->edges[graph->edge_count] = edge;
	graph->edge_count += 1;

	Vertex* from;
	for (int i = 0; i < graph->vertex_count; i++) {
		if (graph->vertices[i]->id == edge->from) {
			from = graph->vertices[i];
			break;
		}
	}

	Vertex* to;
	for (int i = 0; i < graph->vertex_count; i++) {
		if (graph->vertices[i]->id == edge->to) {
			to = graph->vertices[i];
			break;
		}
	}

	Connection* connection = create_connection(to, edge->weight);
	push_linked_list(from->connections, connection);

	connection = create_connection(from, edge->weight);
	push_linked_list(to->connections, connection);
}

/* free_graph
 - frees a graph structure
 - takes the graph to be freed
*/
void free_graph(Graph* graph) {
	if (graph == NULL) { return; }

	for (int i = 0; i < graph->vertex_count; i++) {
		free_vertex(graph->vertices[i]);
	}

	for (int i = 0; i < graph->edge_count; i++) {
		free_edge(graph->edges[i]);
	}

	free(graph);
}

/* create_vertex
 - allocates a new vertex structure
 - takes an int for the id of the vertex
 - takes two doubles for the latitude and longitude of the vertex
 - takes two ints for the earliest and latest times that they are available (should be -1 if not using time based constraints)
 - returns a pointer to the new vertex or NULL on error
*/
Vertex* create_vertex(int id, double lat, double lon, int earliest, int latest) {
	Vertex* vertex = malloc(sizeof(Vertex));
	if (vertex == NULL) { return NULL; }

	vertex->id = id;
	vertex->index = -1;
	vertex->lat = lat;
	vertex->lon = lon;
	vertex->earliest = earliest;
	vertex->latest = latest;
	vertex->connections = create_linked_list();

	return vertex;
}

/* free_vertex
 - frees a vertex structure
 - takes a pointer to the vertex
*/
void free_vertex(Vertex* vertex) {
	free_linked_list(vertex->connections, 1);
	free(vertex);
}

/* create_edge
 - allocates a new edge structure
 - takes two ints for the id of the vertex it is from and to
 - takes a double for the weight of the edge
 - returns a pointer to the new edge or NULL on error
*/
Edge* create_edge(int from, int to, double weight) {
	Edge* edge = malloc(sizeof(Edge));
	if (edge == NULL) { return NULL; }

	edge->from = from;
	edge->to = to;
	edge->weight = weight;

	return edge;
}

/* free_edge
 - frees an edge structure
 - takes a pointer to an edge structure
*/
void free_edge(Edge* edge) {
	free(edge);
}

/* create_connection
 - allocates a new connection structure
 - takes a vertex destination and a double for the weight of the edge
 - returns a pointer to the new connection object or NULL on error
*/
Connection* create_connection(Vertex* vertex, double weight) {
	if (vertex == NULL) { return NULL; }

	Connection* connection = malloc(sizeof(Connection));
	if (connection == NULL) { return NULL; }

	connection->vertex = vertex;
	connection->weight = weight;

	return connection;
}

/* create_path
 - allocates a new path structure
 - returns a pointer to the new path structure or NULL on error
*/
Path* create_path(int nodes_visited, double distance) {
	Path* path = malloc(sizeof(Path));
	if (path == NULL) { return NULL; }

	path->nodes_visited = nodes_visited;
	path->distance = distance;

	path->vertices = create_linked_list();

	return path;
}

/* push_path
 - adds a vertex onto the path
 - takes a path and the vertex to add to it
*/
void push_path(Path* path, Vertex* vertex) {
	if (path == NULL) { return; }
	if (vertex == NULL) { return; }

	push_linked_list(path->vertices, vertex);
}

/* free_path
 - frees a path structure
 - takes a pointer to the path
*/
void free_path(Path* path) {
	if (path == NULL) { return; }

	free_linked_list(path->vertices, 0);
	free(path);
}

/* print_path
 - prints a path structure
 - takes a pointer to the path
*/
void print_path(Path* path) {
	if (path == NULL) { return; }
	if (path->vertices == NULL) { return; }
	if (path->vertices->last == NULL) { return; }

	// line 1
	printf("Path from %d to %d: ", ((Vertex*) path->vertices->last->data)->id, ((Vertex*) path->vertices->first->data)->id);

	Node* current = path->vertices->last;
	while(current != NULL) {
		Vertex* vertex = current->data;
		printf("%d", vertex->id);
		if (current->prev != NULL) { printf(" -> "); }
		current = current->prev;
	}
	printf("\n");

	// line 2
	printf("Total distance: %lf km\n", path->distance);

	// line 3
	printf("Nodes explored: %d\n", path->nodes_visited);
}

/* dijkstra
 - does dijkstra pathfinding without other constraints
 - takes a graph, a start vertex, and an end vertex
 - returns a path structure with the shortest path
*/
Path* dijkstra(Graph* graph, Vertex* start, Vertex* end) {
	PriorityQueue* queue = create_priority_queue(); // contains Vertex*
	
	double* distance = malloc(sizeof(double) * graph->vertex_count);
	int* previous = malloc(sizeof(int) * graph->vertex_count);
	int nodes_visited = 0;

	for (int i = 0; i < graph->vertex_count; i++) {
		distance[i] = -1;
		previous[i] = -1;
	}

	distance[0] = 0;
	previous[0] = 0;

	push_priority_queue(queue, start, 0);

	while (!priority_queue_is_empty(queue)) {
		Vertex* current = pull_priority_queue(queue);
		nodes_visited += 1;

		if (current == end) { break; }

		Node* connection = current->connections->first;
		while (connection != NULL) {
			Connection* data = connection->data;
			int index = ((Vertex*) data->vertex)->index;

			if (previous[index] != -1) { connection = connection->next; continue; }   

			distance[index] = distance[current->index] + data->weight;
			previous[index] = current->id;

			push_priority_queue(queue, data->vertex, distance[index]);

			connection = connection->next;
		}
	}

	Path* path = create_path(nodes_visited, distance[end->index]);

	Vertex* current = end;
	while (current != start) {
		push_path(path, current);

		int index = current->index;

		int previous_id = previous[index];
		Vertex* previous = NULL;
		for (int i = 0; i < graph->vertex_count; i++) {
			if (graph->vertices[i]->id == previous_id) {
				previous = graph->vertices[i];
				break;
			}
		}

		current = previous;
	}

	push_path(path, current);

	free_priority_queue(queue);
	free(distance);
	free(previous);

	return path;
}

int main(int argc, char* argv[]) {
    if (argc != 6) {
        printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>\n", argv[0]);
        printf("Algorithms: dijkstra\n");
        return 1;
    }

	char* vertices_file = argv[1];
	char* edges_file = argv[2];
	int start_node_index = atoi(argv[3]);
	int end_node_index = atoi(argv[4]);
	//char* algorithm = argv[5];

	Graph* graph = create_graph();

	char line[256];

	FILE* fp = fopen(vertices_file, "r");
	if (!fp) { printf("Error opening vertex file\n"); return 1; }

	fgets(line, sizeof(line), fp);
	while (fgets(line, sizeof(line), fp)) {
		int id, earliest, latest;
		double lat, lon;

		int matched = sscanf(line, "%d,%lf,%lf,%d,%d", &id, &lat, &lon, &earliest, &latest);
		if (matched == 3) {
			Vertex* vertex = create_vertex(id, lat, lon, -1, -1);
			add_vertex(graph, vertex);
		} else if (matched == 5) {
			Vertex* vertex = create_vertex(id, lat, lon, earliest, latest);
			add_vertex(graph, vertex);
		}
	}
	fclose(fp);

	fp = fopen(edges_file, "r");
	if (!fp) { printf("Error opening edge file\n"); return 1; }

	fgets(line, sizeof(line), fp);
	while (fgets(line, sizeof(line), fp)) {
		int from, to;
		double distance;

		int matched = sscanf(line, "%d,%d,%lf", &from, &to, &distance);
		if (matched == 3) {
			Edge* edge = create_edge(from, to, distance);
			add_edge(graph, edge);
		}
	}
	fclose(fp);

	Vertex* start_vertex = NULL;
	for (int i = 0; i < graph->vertex_count; i++) {
		if (graph->vertices[i]->id == start_node_index) {
			start_vertex = graph->vertices[i];
			break;
		}
	}

	Vertex* end_vertex = NULL;
	for (int i = 0; i < graph->vertex_count; i++) {
		if (graph->vertices[i]->id == end_node_index) {
			end_vertex = graph->vertices[i];
			break;
		}
	}

	if (start_vertex == NULL || end_vertex == NULL) { printf("Invalid start or end node\n"); return 1; }

	Path* path = dijkstra(graph, start_vertex, end_vertex);

	printf("=== Dijkstra's Algorithm ===\n");
	print_path(path);

	free_path(path);
	free_graph(graph);
}
