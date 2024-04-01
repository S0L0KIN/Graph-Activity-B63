/*
 * Graph algorithms.
 *
 * Author (of starter code): A. Tafliovich.
 */

#include <limits.h>

#include "graph.h"
#include "minheap.h"

#define NOTHING -1
#define DEBUG 0

typedef struct records {
  int numVertices;    // total number of vertices in the graph
                      // vertex IDs are 0, 1, ..., numVertices-1
  MinHeap* heap;      // priority queue
  bool* finished;     // finished[id] is true iff vertex id is finished
                      //   i.e. no longer in the PQ
  int* predecessors;  // predecessors[id] is the predecessor of vertex id
  Edge* tree;         // keeps edges for the resulting tree
  int numTreeEdges;   // current number of edges in mst
} Records;

/*************************************************************************
 ** Suggested helper functions -- part of starter code
 *************************************************************************/

/* Creates, populates, and returns a MinHeap to be used by Prim's and
 * Dijkstra's algorithms on Graph 'graph' starting from vertex with ID
 * 'startVertex'.
 * Precondition: 'startVertex' is valid in 'graph'
 */
MinHeap* initHeap(Graph* graph, int startVertex) {
  MinHeap *new = newHeap(graph->numVertices); //need space for every vertice

  for (int i=0; i<new->capacity; i++) { //for every node in graph
    insert(new, INT_MAX, graph->vertices[i]->id); //insert with INT_MAX priority (from limit library)
  }

  decreasePriority(new, startVertex, 0); //decrease starting node to 0

  return new; //return
}

/* Creates, populates, and returns all records needed to run Prim's and
 * Dijkstra's algorithms on Graph 'graph' starting from vertex with ID
 * 'startVertex'.
 * Precondition: 'startVertex' is valid in 'graph'
 */
Records* initRecords(Graph* graph, int startVertex) {
  Records *new = (Records*)malloc(sizeof(Records)); //get space
  if (new == NULL) return NULL;
  
  new->numVertices = graph->numVertices; //initialize
  new->numTreeEdges = 0;
  new->heap = initHeap(graph, startVertex); //use init heap function (also loads startVertex up in heap)

  new->finished = (bool*)calloc(new->numVertices, sizeof(bool));
  new->predecessors = (int*)calloc(new->numVertices, sizeof(int));
  new->tree = (Edge*)calloc(new->numVertices, sizeof(Edge));
  if (new->finished == NULL || new->predecessors == NULL || new->tree == NULL) { //no space
    exit(1);
  }

  for(int i=0; i<new->numVertices; i++) { //set/initialize all arrays to default values
    new->finished[i] = false;
    new->predecessors[i] = NOTHING;
    
    new->tree[i].fromVertex = NOTHING;
    new->tree[i].toVertex = NOTHING;
    new->tree[i].weight = NOTHING;
  }

  return new;
}


/* Returns true iff 'heap' is NULL or is empty. */
bool isEmpty(MinHeap* heap) {
  return (heap == NULL || heap->size == 0);
}

/* Prints the status of all current algorithm data: good for debugging. */
void printRecords(Records* records);

/* Add a new edge to records at index ind. */
void addTreeEdge(Records* records, int ind, int fromVertex, int toVertex,
                 int weight) {
  records->tree[ind].fromVertex = fromVertex;
  records->tree[ind].toVertex = toVertex;
  records->tree[ind].weight = weight;
  records->numTreeEdges += 1;
}

/* Creates and returns a path from 'vertex' to 'startVertex' from edges
 * in the distance tree 'distTree'.
 */
EdgeList* makePath(Edge* distTree, int vertex, int startVertex) { //solve this by recursively building a path
  if (vertex == startVertex) return NULL; //end of list OR we have reached the node

  int nextVert = distTree[vertex].toVertex; //follow the path 
  EdgeList *next = makePath(distTree, nextVert, startVertex); //atp next is the path from nextVert to startVertex

  int weight = distTree[vertex].weight - distTree[nextVert].weight; //diff between current and next is weight of edge
  Edge *cur = newEdge(vertex, nextVert, weight); //create the edge
  return newEdgeList(cur, next); //add the edge to the list
}

// check if a vertex is valid for a given graph
bool isValidVertex(Graph* graph, int vertex) {
  if (vertex >= 0 && vertex < graph->numVertices) return true;
  return false;
}

//free memory space of record
void deleteRecord(Records *record) {
  deleteHeap(record->heap); //can use helper in minheap.c
  free(record->finished);
  free(record->predecessors);
  free(record->tree);
  free(record);
}

/*************************************************************************
 ** Required functions
 *************************************************************************/

/* Runs Prim's algorithm on Graph 'graph' starting from vertex with ID
 * 'startVertex', and return the resulting MST: an array of Edges.
 * Returns NULL is 'startVertex' is not valid in 'graph'.
 * Precondition: 'graph' is connected.
 */
Edge* getMSTprim(Graph* graph, int startVertex){
  if (!isValidVertex(graph, startVertex)) return NULL; //check valid start point
  Records *record = initRecords(graph, startVertex); //initialize record
  HeapNode min; //for iterating
  EdgeList *adj;

  while(!isEmpty(record->heap)) { // for entire heap
    min = extractMin(record->heap); //take current smallest
    record->finished[min.id] = true; //set extracted to finished
    addTreeEdge(record, min.id, min.id, record->predecessors[min.id], min.priority); //add this edge

    adj = graph->vertices[min.id]->adjList; //get adjacency list for min
    while (adj != NULL) { //for each item in adj list
      if(record->finished[adj->edge->toVertex] == false) { //this node was not found already
        if (decreasePriority(record->heap, adj->edge->toVertex, adj->edge->weight)) { //helper will only change if less, return true if changed
          record->predecessors[adj->edge->toVertex] = min.id; //change predecessor
        } 
      }
      adj = adj->next; //next in list
    }
  }

  Edge *mst = (Edge*)calloc(record->numTreeEdges, sizeof(Edge)); //know how many edges we created
  for(int i=0; i<record->numTreeEdges; i++) { //put in list
    mst[i].fromVertex = record->tree[i].fromVertex;
    mst[i].toVertex = record->tree[i].toVertex;
    mst[i].weight = record->tree[i].weight;
  }
  deleteRecord(record);
  return mst; //free and return
}

/* Runs Dijkstra's algorithm on Graph 'graph' starting from vertex with ID
 * 'startVertex', and return the resulting distance tree: an array of edges.
 * Returns NULL if 'startVertex' is not valid in 'graph'.
 * Precondition: 'graph' is connected.
 */
Edge* getDistanceTreeDijkstra(Graph* graph, int startVertex) {
  if (!isValidVertex(graph, startVertex)) return NULL; //check valid vertex
  Records *record = initRecords(graph, startVertex); //initialize record
  HeapNode min; //for iterating
  EdgeList *adj;
  int curweight = 0;

  while (!isEmpty(record->heap)){
    min = extractMin(record->heap); //extract
    record->finished[min.id] = true; //this id is finished
    addTreeEdge(record, min.id, min.id, record->predecessors[min.id], min.priority); //add this edge
    adj = graph->vertices[min.id]->adjList; //get adj list

    while (adj != NULL) { //for each item in adj list
      curweight = min.priority + adj->edge->weight; //get weight to current edge
      if(!record->finished[adj->edge->toVertex]) { //check if node is finished
        if (decreasePriority(record->heap, adj->edge->toVertex, curweight)) { //helper will only change if less, return true if changed
          record->predecessors[adj->edge->toVertex] = min.id; //change predecessor
        } 
      }
      adj = adj->next;
    }
  }

  Edge *dt = (Edge*)calloc(record->numTreeEdges, sizeof(Edge)); //know how many edges we created
  for(int i=0; i<record->numTreeEdges; i++) { //put in list
    dt[i].fromVertex = record->tree[i].fromVertex;
    dt[i].toVertex = record->tree[i].toVertex;
    dt[i].weight = record->tree[i].weight;
  }
  deleteRecord(record);
  return dt; //free and return
}

/* Creates and returns an array 'paths' of shortest paths from every vertex
 * in the graph to vertex 'startVertex', based on the information in the
 * distance tree 'distTree' produced by Dijkstra's algorithm on a graph with
 * 'numVertices' vertices and with the start vertex 'startVertex'.  paths[id]
 * is the list of edges of the form
 *   [(id -- id_1, w_0), (id_1 -- id_2, w_1), ..., (id_n -- start, w_n)]
 *   where w_0 + w_1 + ... + w_n = distance(id)
 * Returns NULL if 'startVertex' is not valid in 'distTree'.
 */
EdgeList** getShortestPaths(Edge* distTree, int numVertices, int startVertex) {
  if (startVertex < 0 || startVertex >= numVertices) return NULL; //check valid

  EdgeList **paths = (EdgeList**)calloc(numVertices, sizeof(EdgeList*)); //array of our path edge lists

  for(int i=0; i<numVertices; i++) { //for each id from 0 to numVertices, make the path.
    paths[i] = makePath(distTree, i, startVertex);
  }
  return paths;
}

/*************************************************************************
 ** Provided helper functions -- part of starter code to help you debug!
 *************************************************************************/
void printRecords(Records* records) {
  if (records == NULL) return;

  int numVertices = records->numVertices;
  printf("Reporting on algorithm's records on %d vertices...\n", numVertices);

  printf("The PQ is:\n");
  printHeap(records->heap);

  printf("The finished array is:\n");
  for (int i = 0; i < numVertices; i++)
    printf("\t%d: %d\n", i, records->finished[i]);

  printf("The predecessors array is:\n");
  for (int i = 0; i < numVertices; i++)
    printf("\t%d: %d\n", i, records->predecessors[i]);

  printf("The TREE edges are:\n");
  for (int i = 0; i < records->numTreeEdges; i++) printEdge(&records->tree[i]);

  printf("... done.\n");
}
