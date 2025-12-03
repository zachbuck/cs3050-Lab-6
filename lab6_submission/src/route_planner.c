
#include <stdlib.h>
#include <stdio.h>

#include "route_planner.h"

Node* create_node(void* data) {
	Node* node = malloc(sizeof(Node));
	if (node == NULL) { return NULL; }

	node->data = data;
	node->next = NULL;
	node->prev = NULL;

	return node;
}

void* free_node(Node* node) {
	if (node == NULL) { return NULL; }

	void* data = node->data;
	free(node);

	return data;
}

LinkedList* create_linked_list() {
	LinkedList* list = malloc(sizeof(LinkedList));
	if (list == NULL) { return NULL; }

	list->first = NULL;
	list->last = NULL;

	return list;
}

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

PriorityNode* create_priority_node(void* data, double priority) {
	PriorityNode* node = malloc(sizeof(PriorityNode));
	if (node == NULL) { return NULL; }

	node->data = data;
	node->priority = priority;
	node->next = NULL;

	return node;
}

void* free_priority_node(PriorityNode* node) {
	if (node == NULL) { return NULL; }

	void* data = node->data;
	free(node);
	
	return data;
}

PriorityQueue* create_priority_queue() {
	PriorityQueue* queue = malloc(sizeof(PriorityQueue));
	if (queue == NULL) { return NULL; }

	queue->first = NULL;

	return queue;
}

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

void* pull_priority_queue(PriorityQueue* queue) {
	if (queue == NULL) { return NULL; }
	if (queue->first == NULL) { return NULL; }

	PriorityNode* node = queue->first;
	queue->first = node->next;

	void* data = node->data;
	free_priority_node(node);

	return data;
}

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

void add_vertex(Graph* graph, Vertex* vertex) {
	if (graph == NULL) { return; }
	if (vertex == NULL) { return; }

	graph->vertices[graph->vertex_count] = vertex;
	graph->vertex_count += 1;
}

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

Vertex* create_vertex(int id, double lat, double lon, int earliest, int latest) {
	Vertex* vertex = malloc(sizeof(Vertex));
	if (vertex == NULL) { return NULL; }

	vertex->id = id;
	vertex->lat = lat;
	vertex->lon = lon;
	vertex->earliest = earliest;
	vertex->latest = latest;
	vertex->connections = create_linked_list();

	return vertex;
}

void free_vertex(Vertex* vertex) {
	free_linked_list(vertex->connections, 1);
	free(vertex);
}

Edge* create_edge(int from, int to, double weight) {
	Edge* edge = malloc(sizeof(Edge));
	if (edge == NULL) { return NULL; }

	edge->from = from;
	edge->to = to;
	edge->weight = weight;

	return edge;
}

void free_edge(Edge* edge) {
	free(edge);
}

Connection* create_connection(Vertex* vertex, double weight) {
	if (vertex == NULL) { return NULL; }

	Connection* connection = malloc(sizeof(Connection));
	if (connection == NULL) { return NULL; }

	connection->vertex = vertex;
	connection->weight = weight;

	return connection;
}

void dijkstra(Graph* graph, Vertex* start, Vertex* end, double** distances, int** previous, int* nodes_explored) {
	PriorityQueue* queue = create_priority_queue();

	*distances = malloc(sizeof(double) * graph->vertex_count);
	*previous = malloc(sizeof(int) * graph->vertex_count);
	
}

int main(int argc, char* argv[]) {
    if (argc != 6) {
        printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>\n", argv[0]);
        printf("Algorithms: dijkstra, astar, bellman-ford\n");
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

	free_graph(graph);
}
