/*
 * Our graph implementation.
 *
 * Author: A. Tafliovich.
 */

#include "graph.h"

/*********************************************************************
 ** Helper function provided in the starter code
 *********************************************************************/

void printEdge(Edge* edge) {
  if (edge == NULL)
    printf("NULL");
  else
    printf("(%d -- %d, %d)", edge->fromVertex, edge->toVertex, edge->weight);
}

void printEdgeList(EdgeList* head) {
  while (head != NULL) {
    printEdge(head->edge);
    printf(" --> ");
    head = head->next;
  }
  printf("NULL");
}

void printVertex(Vertex* vertex) {
  if (vertex == NULL) {
    printf("NULL");
  } else {
    printf("%d: ", vertex->id);
    printEdgeList(vertex->adjList);
  }
}

void printGraph(Graph* graph) {
  if (graph == NULL) {
    printf("NULL");
    return;
  }
  printf("Number of vertices: %d. Number of edges: %d.\n\n", graph->numVertices,
         graph->numEdges);

  for (int i = 0; i < graph->numVertices; i++) {
    printVertex(graph->vertices[i]);
    printf("\n");
  }
  printf("\n");
}

/*********************************************************************
 ** Required functions
 *********************************************************************/

/* Returns a newly created Edge from vertex with ID 'fromVertex' to vertex
 * with ID 'toVertex', with weight 'weight'.
 */
Edge* newEdge(int fromVertex, int toVertex, int weight) {
  Edge *new = (Edge*)malloc(sizeof(Edge)); //allocate on the heap
  if (new == NULL) return NULL; //no space
  new->fromVertex = fromVertex; //initialize
  new->toVertex = toVertex;
  new->weight=weight;
  return new; //return
}

/* Returns a newly created EdgeList containing 'edge' and pointing to the next
 * EdgeList node 'next'.
 */
EdgeList* newEdgeList(Edge* edge, EdgeList* next) {
  EdgeList *new = (EdgeList*)malloc(sizeof(EdgeList)); //allocate on heap
  if (new == NULL) return NULL; //no space
  new->edge = edge; //initialize
  new->next = next;
  return new; //return
}

/* Returns a newly created Vertex with ID 'id', value 'value', and adjacency
 * list 'adjList'.
 * Precondition: 'id' is valid for this vertex
 */
Vertex* newVertex(int id, void* value, EdgeList* adjList) {
  Vertex *new = (Vertex*)malloc(sizeof(Vertex)); //allocate on heap
  if (new == NULL) return NULL; //no space
  new->id = id; //initialize
  new->value = value;
  new->adjList = adjList;
  return new; //return
}

/* Returns a newly created Graph with space for 'numVertices' vertices.
 * Precondition: numVertices >= 0
 */
Graph* newGraph(int numVertices) {
  Graph *new = (Graph*)malloc(sizeof(Graph)); //allocate on heap
  if (new == NULL) return NULL; //no space
  new->numEdges = 0; //initialize
  new->numVertices = numVertices;
  if (numVertices == 0) new->vertices = NULL; //cant calloc block of 0 size
  else {
    new->vertices = (Vertex**)calloc(numVertices, sizeof(Vertex)); //list of vertices is pointers on heap
  }
  return new;
}

/* Frees memory allocated for EdgeList starting at 'head'.
 */
void deleteEdgeList(EdgeList* head) {
  if (head != NULL) {
    EdgeList* temp = head->next; //store next value
    free(head->edge);
    free(head); //free current
    deleteEdgeList(temp); //if next value exists, delete it recursively
  }
}

/* Frees memory allocated for 'vertex' including its adjacency list.
 */
void deleteVertex(Vertex* vertex) {
  deleteEdgeList(vertex->adjList); //delete the vertex's adjlist
  free(vertex); //free vertex
}

/* Frees memory allocated for 'graph'.
 */
void deleteGraph(Graph* graph) {
  for (int i=0; i<graph->numVertices; i++) { //for each node (ids go from 0 to numVertices-1)
    deleteVertex(graph->vertices[i]); //remove it and its list
  }
  free(graph->vertices);
  free(graph); //free graph
}