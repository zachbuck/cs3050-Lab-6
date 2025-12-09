
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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

/* find_vertex_with_id
 - finds a pointer to a vertex with a certain id
 - takes a graph to search and an id to search for
 - returns a pointer to the vertex or NULL on error
*/
Vertex* find_vertex_with_id(Graph* graph, int id) {
	if (graph == NULL) { return NULL; }

	for (int i = 0; i < graph->vertex_count; i++) {
		if (graph->vertices[i]->id == id) {
			return graph->vertices[i];
		}
	}

	return NULL;
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
	if (vertex == NULL) { return; }

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
	if (edge == NULL) { return; }

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

	free_linked_list(path->vertices, path_is_error(path));
	free(path);
}

/* print_path
 - prints a path structure
 - takes a pointer to the path
*/
void print_path(Path* path) {
	if (path == NULL) { return; }
	if (path_is_error(path)) { print_path_error(path); return; }
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

/* print_path_error
 - prints an error message in an invalid path
 - takes a pointer to the path
*/
void print_path_error(Path* path) {
	Node* current = path->vertices->first;
	while (current != NULL) {
		printf("%s\n", ((char*) current->data));
		current = current->next;
	}
}

/* path_is_error
 - returns if a path is an error from an invalid path or a valid path
 - takes a pointer to the path
 - returns 0 for a valid path and != 0 for any invalid path
*/
int path_is_error(Path* path) {
	return path->distance == -1 && path->nodes_visited == -1;
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

	distance[start->index] = 0;

	push_priority_queue(queue, start, 0);

	while (!priority_queue_is_empty(queue)) {
		// pull the closest unvisited vertex
		Vertex* current = pull_priority_queue(queue);
		nodes_visited += 1;

		// if the current vertex is the end goal, stop searching
		if (current == end) { break; }

		// search through all adjacent vertices
		Node* connection = current->connections->first;
		while (connection != NULL) {
			Connection* data = connection->data;
			int index = ((Vertex*) data->vertex)->index;

			// since previous[index] is set to != -1 when it is found this basically just skips vertices that have already been identified
			if (previous[index] != -1) { connection = connection->next; continue; }   

			distance[index] = distance[current->index] + data->weight;
			previous[index] = current->id;

			push_priority_queue(queue, data->vertex, distance[index]);

			connection = connection->next;
		}
	}

	// on not finding a viable path
	if (previous[end->index] == -1) { 
		Path* path = create_path(-1, -1);
		char error_message[] = "Could not find viable path";
		char* heap_error = malloc(sizeof(error_message));
		memcpy(heap_error, error_message, sizeof(error_message));
		push_linked_list(path->vertices, heap_error);

		free_priority_queue(queue);
		free(distance);
		free(previous);

		return path;
	}

	Path* path = create_path(nodes_visited, distance[end->index]);

	// trace back the fastest route identified and push them into the path
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

	// free allocated temporary resources and return the identified path
	free_priority_queue(queue);
	free(distance);
	free(previous);

	return path;
}

/* time_constrained_dijkstra
 - does dijkstra pathfinding taking into account the earliest and latest values of each of the vertices
 - takes a graph, a start vertex, and an end vertex
 - returns a path structure with the shortest path or an error path on error
*/
Path* time_constrained_dijkstra(Graph* graph, Vertex* start, Vertex* end) {
	PriorityQueue* queue = create_priority_queue(); // contains Vertex*
	
	double* distance = malloc(sizeof(double) * graph->vertex_count);
	int* previous = malloc(sizeof(int) * graph->vertex_count);
	int nodes_visited = 0;

	for (int i = 0; i < graph->vertex_count; i++) {
		distance[i] = -1;
		previous[i] = -1;
	}

	distance[start->index] = 0;

	push_priority_queue(queue, start, 0);

	// This works largely identically to my basic dijkstra so I will comment differences
	while (!priority_queue_is_empty(queue)) {
		Vertex* current = pull_priority_queue(queue);
		nodes_visited += 1;
		double current_distance = distance[current->index];

		if (current == end) { break; }

		Node* connection = current->connections->first;
		while (connection != NULL) {
			Connection* data = connection->data;
			int index = ((Vertex*) data->vertex)->index;

			if (previous[index] != -1) { connection = connection->next; continue; }
			
			// ignore connections that cannot be accessed since the earliest and latest windows don't match up
			if (current->latest + data->weight < ((Vertex*) data->vertex)->earliest) { connection = connection->next; continue; }
			if (current_distance + data->weight > ((Vertex*) data->vertex)->latest) { connection = connection->next; continue; }

			// wait if necessary for the vertex to become available
			if (current_distance + data->weight > ((Vertex*) data->vertex)->earliest) {
				distance[index] = current_distance + data->weight;
				previous[index] = current->id;
			} else {
				distance[index] = ((Vertex*) data->vertex)->earliest;
				previous[index] = current->id;
			}

			push_priority_queue(queue, data->vertex, distance[index]);

			connection = connection->next;
		}
	}

	// on not finding a viable path
	if (previous[end->index] == -1) { 
		Path* path = create_path(-1, -1);
		char error_message[] = "Could not find viable path";
		char* heap_error = malloc(sizeof(error_message));
		memcpy(heap_error, error_message, sizeof(error_message));
		push_linked_list(path->vertices, heap_error);

		free_priority_queue(queue);
		free(distance);
		free(previous);

		return path;
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

/* find_optimal_multi_route_recursion
 - does a recursive search through a graph for the multiroute through it's vertices
 - takes a fully saturated graph, a start id, an array of remaining ids, and the length of that array
 - returns the shortest path through the remaining ids
*/
Path* find_optimal_multi_route_recursion(Graph* graph, int start, int* ids, int id_count) {
	if (graph == NULL) { return NULL; }
	if (ids == NULL) { return NULL; }

	// if the recursion is complete and there is only one path availabe, return that path
	if (id_count == 1) {
		Vertex* first = find_vertex_with_id(graph, start);
		Vertex* last = find_vertex_with_id(graph, *ids);

		// find the connection to the remaining vertex
		// this is guaranteed to exist since Graph* graph, must be a fully connected graph
		double weight = 0;
		Node* current = first->connections->first;
		while (current != NULL) {
			Connection* connection = current->data;

			if (connection->vertex == last) {
				weight = connection->weight;
				break;
			}

			current = current->next;
		}

		// create and return the one and only one path that can exist at this state of the recursion
		Path* path = create_path(0, weight);
		push_path(path, last);
		push_path(path, first);
		
		return path;
	}

	// search through the space of all current steps that can be taken and keep the one that results in the best path
	Path* best = NULL;
	for (int i = 0; i < id_count; i++) {
		// populate the array 'other_ids'
		int* other_ids = malloc(sizeof(int) * id_count - 1);
		for (int j = 0; j < id_count; j++) {
			if (j == i) { continue; }
			int index = j < i ? j : j - 1;

			other_ids[index] = ids[j];
		}

		// find the best route through all of the remaining ids and either replace best or free the path
		Path* current = find_optimal_multi_route_recursion(graph, ids[i], other_ids, id_count - 1);
		if (best == NULL) { 
			best = current; 
		} else if (current->distance < best->distance) { 
			free_path(best); 
			best = current; 
		} else {
			free_path(current);
		}

		free(other_ids);
	}

	//add the remaining connection information into the path and return it
	Vertex* first = find_vertex_with_id(graph, start);
	Vertex* end = best->vertices->last->data;
	double weight = 0;
	Node* current = first->connections->first;
	while (current != NULL) {
		Connection* connection = current->data;

		if (connection->vertex == end) {
			weight = connection->weight;
		}

		current = current->next;
	}

	best->distance += weight;
	push_path(best, first);

	return best;
}

/* find_optimal_multi_route
 - searches through a graph for the fastest route containing a list of points
 - takes a graph to search through and a linked list of vertices to route through
 - returns the fastest path through those points
*/
Path* find_optimal_multi_route(Graph* graph, LinkedList* vertices) {
	if (graph == NULL) { return NULL; }
	if (vertices == NULL) { return NULL; }

	// this is a second graph used internally in this function
	// it contains only those vertices that are being pathed between and is fully connected 
	// with edges weighted to the fastest route between those vertices on the original graph
	Graph* internal = create_graph();

	// copy all needed vertices into the graph
	Node* current = vertices->first;
	while (current != NULL) {
		Vertex* v = current->data;
		Vertex* copy = create_vertex(v->id, v->lat, v->lon, v->earliest, v->latest);
		add_vertex(internal, copy);
		current = current->next;
	}

	// create all of the needed edges in the graph
	for (int i = 0; i < internal->vertex_count; i++) {
		for (int j = i + 1; j < internal->vertex_count; j++) {
			Vertex* start = NULL;
			for (int x = 0; x < graph->vertex_count; x++) {
				if (graph->vertices[x]->id == internal->vertices[i]->id) {
					start = graph->vertices[x];
					break;
				}
			}

			Vertex* end = NULL;
			for (int x = 0; x < graph->vertex_count; x++) {
				if (graph->vertices[x]->id == internal->vertices[j]->id) {
					end = graph->vertices[x];
					break;
				}
			}

			Path* path = dijkstra(graph, start, end);
			
			Edge* edge = create_edge(internal->vertices[i]->id, internal->vertices[j]->id, path->distance);

			add_edge(internal, edge);

			free_path(path);
		}
	}

	// iterate through all vertices one could start with and use the recursive method to find the best route starting at each one
	// this works in the same way as the recursive version
	Path* best = NULL;
	for (int i = 0; i < internal->vertex_count; i++) {
		int* other_vertices = malloc(sizeof(int) * internal->vertex_count - 1);
		for (int j = 0; j < internal->vertex_count; j++) {
			if (j == i) { continue; }
			int index = j < i ? j : j - 1;

			other_vertices[index] = internal->vertices[j]->id;
		}

		Path* current = find_optimal_multi_route_recursion(internal, internal->vertices[i]->id, other_vertices, internal->vertex_count - 1);
		if (best == NULL) { 
			best = current; 
		} else if (current->distance < best->distance) { 
			free_path(best); 
			best = current; 
		} else {
			free_path(current);
		}

		free(other_vertices);
	}

	// create a new path with the vertices from the original graph
	Path* out = create_path(0, best->distance);
	Node* node = best->vertices->last;
	Node* next = best->vertices->last->prev;
	while (next != NULL) {
		Vertex* v1 = node->data;
		Vertex* v2 = next->data;

		Vertex* v3 = find_vertex_with_id(graph, v1->id);
		Vertex* v4 = find_vertex_with_id(graph, v2->id);

		Path* path = dijkstra(graph, v3, v4);
		Node* current = path->vertices->last;
		while (current->prev != NULL) {
			Vertex* vertex = current->data;
			push_path(out, vertex);
			current = current->prev;
		}

		free_path(path);

		node = next;
		next = next->prev;
	}
	push_path(out, find_vertex_with_id(graph, ((Vertex*) node->data)->id));

	free_path(best);
	free_graph(internal);
	return out;
}

/* find_optimal_priority_multi_route_recursion
 - does a recursive search through a graph for the multiroute through it's vertices, going to the most prioritized first
 - takes a fully saturated graph, a start id, an array of remaining ids, an array of priorities for those ids, and the length of both of those arrays
 - returns the shortest path that respects priority through the remaining ids
*/
Path* find_optimal_priority_multi_route_recursion(Graph* graph, int start, int* ids, int* priorities, int id_count) {
	if (graph == NULL) { return NULL; }
	if (ids == NULL) { return NULL; }
	if (priorities == NULL) { return NULL; }

	// this all works essentially identically to the non-priority version so i will comment where differences occur
	if (id_count == 1) {
		Vertex* first = find_vertex_with_id(graph, start);
		Vertex* last = find_vertex_with_id(graph, *ids);

		double weight = 0;
		Node* current = first->connections->first;
		while (current != NULL) {
			Connection* connection = current->data;

			if (connection->vertex == last) {
				weight = connection->weight;
				break;
			}

			current = current->next;
		}

		Path* path = create_path(0, weight);
		push_path(path, last);
		push_path(path, first);

		return path;
	}

	// find the vertex with the lowest priority
	int lowest_prio = -1;
	for (int i = 0; i < id_count; i++) {
		if (lowest_prio == -1) { lowest_prio = priorities[i]; }
		else if (lowest_prio > priorities[i]) { lowest_prio = priorities[i]; }
	}

	
	Path* best = NULL;
	for (int i = 0; i < id_count; i++) {
		// skip next nodes which do not have the lowest priority
		if (priorities[i] != lowest_prio) { continue; }

		int* other_ids = malloc(sizeof(int) * id_count - 1);
		int* other_prios = malloc(sizeof(int) * id_count - 1);
		for (int j = 0; j < id_count; j++) {
			if (j == i) { continue; }
			int index = j < i ? j : j - 1;

			other_ids[index] = ids[j];
			other_prios[index] = priorities[j];
		}

		Path* current = find_optimal_priority_multi_route_recursion(graph, ids[i], other_ids, other_prios, id_count - 1);
		if (best == NULL) {
			best = current;
		} else if (current->distance < best->distance) {
			free_path(best);
			best = current;
		} else {
			free_path(current);
		}

		free(other_ids);
		free(other_prios);
	}

	Vertex* first = find_vertex_with_id(graph, start);
	Vertex* end = best->vertices->last->data;
	double weight = 0;
	Node* current = first->connections->first;
	while (current != NULL) {
		Connection* connection = current->data;

		if (connection->vertex == end) {
			weight = connection->weight;
		}

		current = current->next;
	}

	best->distance += weight;
	push_path(best, first);

	return best;
}

/* find_optimal_priority_multi_route
 - searches through a graph for the fastest route that respects priority through a list of vertices
 - takes a graph to search through, a linked list of vertices, and a linked list of priorities for those vertices
 - returns the fastest path that respects priority through those points
*/
Path* find_optimal_priority_multi_route(Graph* graph, LinkedList* vertices, LinkedList* priority_list) {
	if (graph == NULL) { return NULL; }
	if (vertices == NULL) { return NULL; }
	if (priority_list == NULL) { return NULL; }

	// again this is almost exactly identical to the non-priority version so I will comment where differences exist 
	Graph* internal = create_graph();

	Node* current = vertices->first;
	while (current != NULL) {
		Vertex* v = current->data;
		Vertex* copy = create_vertex(v->id, v->lat, v->lon, v->earliest, v->latest);
		add_vertex(internal, copy);
		current = current->next;
	}

	for (int i = 0; i < internal->vertex_count; i++) {
		for (int j = i + 1; j < internal->vertex_count; j++) {
			Vertex* start = find_vertex_with_id(graph, internal->vertices[i]->id);
			Vertex* end = find_vertex_with_id(graph, internal->vertices[j]->id);

			Path* path = dijkstra(graph, start, end);

			Edge* edge = create_edge(internal->vertices[i]->id, internal->vertices[j]->id, path->distance);
			add_edge(internal, edge);

			free_path(path);
		}
	}

	// create an array to store the priorities
	int* priorities = malloc(sizeof(int) * internal->vertex_count);
	Node* prio = priority_list->first;
	Node* v_node = vertices->first;
	int i = 0;
	while (prio != NULL) {
		priorities[find_vertex_with_id(internal, ((Vertex*) v_node->data)->id)->index] = *((int*) prio->data);
		i++;
		prio = prio->next;
		v_node = v_node->next;
	}

	// find the lowest priority
	int lowest_priority = -1;
	for (int i = 0; i < internal->vertex_count; i++) {
		if (lowest_priority == -1) { lowest_priority = priorities[i]; }
		else if (lowest_priority > priorities[i]) { lowest_priority = priorities[i]; }
	}

	Path* best = NULL;
	for (int i = 0; i < internal->vertex_count; i++) {
		// skip vertices without the lowest priority
		if (priorities[i] != lowest_priority) { continue; }

		int* other_vertices = malloc(sizeof(int) * internal->vertex_count - 1);
		int* other_priorities = malloc(sizeof(int) * internal->vertex_count - 1);
		for (int j = 0; j < internal->vertex_count; j++) {
			if (j == i) { continue; }
			int index = j < i ? j : j - 1;

			other_vertices[index] = internal->vertices[j]->id;
			other_priorities[index] = priorities[j];
		}

		Path* current = find_optimal_priority_multi_route_recursion(internal, internal->vertices[i]->id, other_vertices, other_priorities, internal->vertex_count -1);
		if (best == NULL) {
			best = current;
		} else if (current->distance < best->distance) {
			free_path(best);
			best = current;
		} else {
			free_path(current);
		}

		free(other_vertices);
		free(other_priorities);
	}

	Path* out = create_path(0, best->distance);
	Node* node = best->vertices->last;
	Node* next = best->vertices->last->prev;
	while (next != NULL) {
		Vertex* v1 = node->data;
		Vertex* v2 = next->data;

		Vertex* v3 = find_vertex_with_id(graph, v1->id);
		Vertex* v4 = find_vertex_with_id(graph, v2->id);

		Path* path = dijkstra(graph, v3, v4);
		Node* current = path->vertices->last;
		while (current->prev != NULL) {
			Vertex* vertex = current->data;
			push_path(out, vertex);
			current = current->prev;
		}

		free_path(path);

		node = next;
		next = next->prev;
	}
	push_path(out, find_vertex_with_id(graph, ((Vertex*) node->data)->id));

	free(priorities);
	free_path(best);
	free_graph(internal);

	return out;
}

int main(int argc, char* argv[]) {
	// print various descriptions of the usages of the program
    if (argc < 4) {
        printf("Usage: %s <nodes.csv> <edges.csv> <algorithm> <additional arguments...>\n", argv[0]);
        printf("Algorithms: dijkstra, time_contrained_dijkstra, multi_route, priority_multi_route\n");
        return 1;
    } else {
		char* algorithm = argv[3];
		if (!strcmp(algorithm, "dijkstra") && argc != 6) {
			printf("Usage: %s <node.csv> <edges.csv> dijkstra <start node> <end node>\n", argv[0]);
        	return 1;
		} else if (!strcmp(algorithm, "time_constrained_dijkstra") && argc != 6) {
			printf("Usage: %s <node.csv> <edges.csv> time_constrained_dijkstra <start_node> <end_node>\n", argv[0]);
        	return 1;
		} else if (!strcmp(algorithm, "multi_route") && argc == 4) {
			printf("Usage: %s <node.csv> <edges.csv> multi_route <node 1> <node 2> <node 3> ...\n", argv[0]);
        	return 1;
		} else if (!strcmp(algorithm, "priority_multi_route") && argc == 4) {
			printf("Usage: %s <node.csv> <edges.csv> priority_multi_route <tolerance percent> <node 1>,<priority 1> <node 2>,<priority 2> <node 3>,<priority 3> ...\n", argv[0]);
			printf("<tolerance percent> should be given in decimal format (i.e. 0.20)\n");
			printf("<priority n> should be given as an integer where 1 is the lowest priority followed by 2 etc.\n");
        	return 1;
		} else if (strcmp(algorithm, "dijkstra") && strcmp(algorithm, "time_constrained_dijkstra") && strcmp(algorithm, "multi_route") && strcmp(algorithm, "priority_multi_route")) {
			printf("Usage: %s <nodes.csv> <edges.csv> <algorithm> <additional arguments...>\n", argv[0]);
			printf("Algorithms: dijkstra, time_contrained_dijkstra, multi_route, priority_multi_route\n");
			return 1;
		}
	}

	char* vertices_file = argv[1];
	char* edges_file = argv[2];
	char* algorithm = argv[3];

	// read the graph from the supplied files
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

	if (!strcmp(algorithm, "dijkstra")) {
		Vertex* start = find_vertex_with_id(graph, atoi(argv[4]));
		Vertex* end = find_vertex_with_id(graph, atoi(argv[5]));

		if (start == NULL || end == NULL) {
			printf("Invalid start or end node\n");
			free_graph(graph);
			return 1;
		}

		Path* path = dijkstra(graph, start, end);

		printf("=== Dijkstra ===\n");
		print_path(path);
		
		free_path(path);
	} else if (!strcmp(algorithm, "time_constrained_dijkstra")) {
		Vertex* start = find_vertex_with_id(graph, atoi(argv[4]));
		Vertex* end = find_vertex_with_id(graph, atoi(argv[5]));

		if (start == NULL || end == NULL) {
			printf("Invalid start or end node\n");
			free_graph(graph);
			return 1;
		}

		Path* path = time_constrained_dijkstra(graph, start, end);
		
		printf("=== Time Constrained Dijkstra ===\n");
		print_path(path);

		free_path(path);
	} else if (!strcmp(algorithm, "multi_route")) {
		LinkedList* vertices = create_linked_list();
		for (int i = 4; i < argc; i++) {
			Vertex* current = find_vertex_with_id(graph, atoi(argv[i]));
			push_linked_list(vertices, current);
		}

		Path* path = find_optimal_multi_route(graph, vertices);

		free_linked_list(vertices, 0);

		printf("=== Multi Route ===\n");
		print_path(path);

		free_path(path);
	} else if (!strcmp(algorithm, "priority_multi_route")) {
		LinkedList* vertices = create_linked_list();
		LinkedList* priorities = create_linked_list();
		for (int i = 5; i < argc; i++) {
			int id, priority;
			if (sscanf(argv[i], "%d,%d", &id, &priority) != 2) {
				free_linked_list(vertices, 0);
				free_linked_list(priorities, 1);
				free_graph(graph);
				return 1;
			}

			Vertex* vertex = find_vertex_with_id(graph, id);
			push_linked_list(vertices, vertex);

			int* prio = malloc(sizeof(int));
			*prio = priority;
			push_linked_list(priorities, prio);
		}

		Path* prio_route = find_optimal_priority_multi_route(graph, vertices, priorities);
		Path* non_prio_route = find_optimal_multi_route(graph, vertices);

		printf("=== Priority multi route ===\n");
		printf("=== Path that follows priority ===\n");
		print_path(prio_route);

		if (non_prio_route->distance < (1 - atof(argv[4])) * prio_route->distance) {
			printf("=== Faster route that breaks priority ===\n");
			printf("=== Distance is %f%% as long ===\n", non_prio_route->distance / prio_route->distance * 100);
			print_path(non_prio_route);
		}

		free_linked_list(vertices, 0);
		free_linked_list(priorities, 1);
		free_path(prio_route);
		free_path(non_prio_route);
	}

	free_graph(graph);
}
