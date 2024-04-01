/*
 * Our min-heap implementation.
 *
 * Author (starter code): A. Tafliovich.
 */

#include "minheap.h"

#define ROOT_INDEX 1
#define NOTHING -1

/*************************************************************************
 ** Suggested helper functions -- to help designing your code
 *************************************************************************/

/* Returns True if 'maybeIdx' is a valid index in minheap 'heap', and 'heap'
 * stores an element at that index. Returns False otherwise.
 */
bool isValidIndex(MinHeap* heap, int maybeIdx) {
       if (maybeIdx < 1 || maybeIdx > heap-> size) return false;
       return true;
}

/* Returns priority of node at index 'nodeIndex' in minheap 'heap'.
 * Precondition: 'nodeIndex' is a valid index in 'heap'
 *               'heap' is non-empty
 */
int priorityAt(MinHeap* heap, int nodeIndex) {
       if (!isValidIndex(heap, nodeIndex)) return NOTHING;
       return heap->arr[nodeIndex].priority;
}

/* Returns node at index 'nodeIndex' in minheap 'heap'.
 * Precondition: 'nodeIndex' is a valid index in 'heap'
 *               'heap' is non-empty
 */
HeapNode nodeAt(MinHeap* heap, int nodeIndex){
       return heap->arr[nodeIndex];
}

/* Returns index of node with ID 'id' in minheap 'heap'.
 * Precondition: 'id' is a valid ID in 'heap'
 *               'heap' is non-empty
 */
int indexOf(MinHeap* heap, int id) {
       return heap->indexMap[id];
}

/* Returns ID of node at index 'nodeIndex' in minheap 'heap'.
 * Precondition: 'nodeIndex' is a valid index in 'heap'
 *               'heap' is non-empty
 */
int idAt(MinHeap* heap, int nodeIndex) {
       if (!isValidIndex(heap, nodeIndex)) return NOTHING;
       return heap->arr[nodeIndex].id;
}

/* Returns the index of the left child of a node at index 'nodeIndex' in
 * minheap 'heap', if such exists.  Returns NOTHING if there is no such left
 * child.
 */
int leftIdx(MinHeap* heap, int nodeIndex) {
       if (!isValidIndex(heap, (2*nodeIndex))) return NOTHING;
       return 2*nodeIndex;
}

/* Returns the index of the right child of a node at index 'nodeIndex' in
 * minheap 'heap', if such exists.  Returns NOTHING if there is no such right
 * child.
 */
int rightIdx(MinHeap* heap, int nodeIndex) {
       if (!isValidIndex(heap, (2*nodeIndex +1))) return NOTHING;
       return 2*nodeIndex +1;
}

/* Returns the index of the parent of a node at index 'nodeIndex' in minheap
 * 'heap', if such exists.  Returns NOTHING if there is no such parent.
 */
int parentIdx(MinHeap* heap, int nodeIndex) {
       if (!isValidIndex(heap, (nodeIndex/2))) return NOTHING;
       return nodeIndex/2;
}


/* Swaps contents of heap->arr[index1] and heap->arr[index2] if both 'index1'
 * and 'index2' are valid indices for minheap 'heap'. Has no effect
 * otherwise.
 */
void swap(MinHeap* heap, int index1, int index2) {
       if (!isValidIndex(heap, index1) || !isValidIndex(heap, index2)) return; // index invalid
       int tempID = idAt(heap, index1); //create temp variables
       int tempPrio = priorityAt(heap, index1);

       heap->indexMap[tempID] = index2; //update index map
       heap->indexMap[idAt(heap, index2)] = index1; 

       heap->arr[index1].id = idAt(heap, index2); //change values of index1
       heap->arr[index1].priority = priorityAt(heap, index2);

       heap->arr[index2].id = tempID; //plug copied values into index2
       heap->arr[index2].priority = tempPrio;
}

/* Bubbles up the element newly inserted into minheap 'heap' at index
 * 'nodeIndex', if 'nodeIndex' is a valid index for heap. Has no effect
 * otherwise.
 */
void bubbleUp(MinHeap* heap, int nodeIndex) {
       if (!isValidIndex(heap, nodeIndex)) return; // index invalid
       if (nodeIndex == ROOT_INDEX) return; //index already bubbled to top

       int parIndex = parentIdx(heap, nodeIndex); //get parent index
       if (parIndex == NOTHING) return; //could not find parent index

       if (priorityAt(heap, nodeIndex) < priorityAt(heap, parIndex)) { //check if smaller priority than parent
              swap(heap, nodeIndex, (parIndex)); //swap values
              bubbleUp(heap, (parIndex)); //recursively bubble parent
       }
}

/* Bubbles down the element newly inserted into minheap 'heap' at the root,
 * if it exists. Has no effect otherwise.
 */
void bubbleDown(MinHeap* heap) {
       if (!isValidIndex(heap, ROOT_INDEX)) return; //check valid
       int curRoot = ROOT_INDEX; //need to initialize outside of for loop
       int left = NOTHING;
       int right = NOTHING;

       while (true) {
              left = leftIdx(heap, curRoot);
              right = rightIdx(heap, curRoot);
              if(left == NOTHING && right == NOTHING) return; //there is nothing to swap

              else if(left == NOTHING && priorityAt(heap,curRoot) < priorityAt(heap,right)) return; //case where right is bigger and left is null
              else if(left == NOTHING && priorityAt(heap,curRoot) > priorityAt(heap,right)) { //null node and swap made
                     swap(heap, curRoot, right);
                     curRoot = right;
              }

              else if(right == NOTHING && priorityAt(heap,curRoot) < priorityAt(heap,left)) return; //^ symmetrical
              else if(right == NOTHING && priorityAt(heap,curRoot) > priorityAt(heap,left)) { //null node and swap made
                     swap(heap, curRoot, left);
                     curRoot = left;
              }
              //compare both nodes cases
              else if (priorityAt(heap, curRoot) > priorityAt(heap, left) && priorityAt(heap, right) >= priorityAt(heap, left)) { //need to bubble left
                     swap(heap, curRoot, left); //swap and repeat on left side
                     curRoot = left;
              }
              else if(priorityAt(heap, curRoot) > priorityAt(heap, right) && priorityAt(heap, left) >= priorityAt(heap, right)) { //need to bubble right
                     swap(heap, curRoot, right); //swap and repeat on right
                     curRoot = right;
              }

              else return; //no bubbling was done, must be in correct spot
       }
}

/*********************************************************************
 * Required functions
 ********************************************************************/

/* Returns a newly created empty minheap with initial capacity 'capacity'.
 * Precondition: capacity >= 0
 */
MinHeap* newHeap(int capacity) {
       if (capacity < 0) return NULL; //error handle
    
       MinHeap* heap = (MinHeap*)malloc(sizeof(MinHeap));
       heap->size = 0; //initialize
       heap->capacity = capacity;

       heap->arr = (HeapNode*)malloc(sizeof(HeapNode)*(capacity+1)); //allocate space for arrays
       heap->indexMap = (int*)malloc(sizeof(int)*(capacity)); 

       for (int i = 0; i < capacity; i++) { //set index map
              heap->indexMap[i] = NOTHING;
       }

       return heap;
}

/* Frees all memory allocated for minheap 'heap'.
 */
void deleteHeap(MinHeap* heap) {
       if (heap == NULL) return;
       free(heap->arr);
       free(heap->indexMap);
       free(heap);
}

/* Returns the node with minimum priority in minheap 'heap'.
 * Precondition: heap is non-empty
 */
HeapNode getMin(MinHeap* heap) {
       return nodeAt(heap, ROOT_INDEX);
}

/* Removes and returns the node with minimum priority in minheap 'heap'.
 * Precondition: heap is non-empty
 */
HeapNode extractMin(MinHeap* heap) {
       if (heap->size==0) exit(1);
       HeapNode min = nodeAt(heap, ROOT_INDEX); //get node
       int minID = heap->arr[ROOT_INDEX].id;

       swap(heap, ROOT_INDEX, heap->size); //swap bottom to top
       heap->indexMap[minID] = NOTHING; //remove from index map

       heap->size-=1; //decrement by 1, the node remains but is ignored as it is outside of size

       bubbleDown(heap); //will handle bubbling down top node
       
       return min;
}

/* Inserts a new node with priority 'priority' and ID 'id' into minheap 'heap'.
 * Precondition: 'id' is unique within this minheap
 *               0 <= 'id' < heap->capacity
 *               heap->size < heap->capacity
 */
void insert(MinHeap* heap, int priority, int id) {
       HeapNode newNode; //initialize newnode
       newNode.id = id;
       newNode.priority = priority;

       heap->size += 1; //update size
       heap->indexMap[id] = heap->size; //add to index map
       heap->arr[heap->size] = newNode; //add new node
       
       bubbleUp(heap, heap->size); //bubble up
}

/* Sets priority of node with ID 'id' in minheap 'heap' to 'newPriority', if
 * such a node exists in 'heap' and its priority is larger than
 * 'newPriority', and returns True. Has no effect and returns False, otherwise.
 * Note: this function bubbles up the node until the heap property is restored.
 */
bool decreasePriority(MinHeap* heap, int id, int newPriority) {
       int index = heap->indexMap[id]; //get index
       if (!isValidIndex(heap, index)) return false; //error handle 

       if (heap->arr[index].priority > newPriority) { //if bigger then we need to change and bubble up
              heap->arr[index].priority = newPriority;
              bubbleUp(heap, index);
              return true;
       }
       return false; //otherwise no change is required
}      


/*********************************************************************
 ** Helper function provided
 *********************************************************************/
void printHeap(MinHeap* heap) {
  printf("MinHeap with size: %d\n\tcapacity: %d\n\n", heap->size,
         heap->capacity);
  printf("index: priority [ID]\t ID: index\n");
  for (int i = 0; i < heap->capacity; i++)
    printf("%d: %d [%d]\t\t%d: %d\n", i, priorityAt(heap, i), idAt(heap, i), i,
           indexOf(heap, i));
  printf("%d: %d [%d]\t\t\n", heap->capacity, priorityAt(heap, heap->capacity),
         idAt(heap, heap->capacity));
  printf("\n\n");
}
