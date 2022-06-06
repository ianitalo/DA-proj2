#ifndef UNTITLED_MAXHEAP_H
#define UNTITLED_MAXHEAP_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#undef PARENT
#undef RIGHT
#undef LEFT
using namespace std;

// Data structure to store a max-heap node
class PriorityQueue
{
    struct Node { // An element of the heap: a pair (key, value)
        int key;
        int value;
    };
private:
    // vector to store heap elements
    vector<Node> A;
    unordered_map<int, int> pos; // maps a key into its position on the array a
    int s;

    // return parent of `A[i]`
    // don't call this function if `i` is already a root node
    int PARENT(int i) {
        return (i - 1) / 2;
    }

    // return left child of `A[i]`
    int LEFT(int i) {
        return (2*i + 1);
    }

    // return right child of `A[i]`
    int RIGHT(int i) {
        return (2*i + 2);
    }

    // Recursive heapify-down algorithm.
    // The node at index `i` and its two direct children
    // violates the heap property
    void heapify_down(int i)
    {
        // get left and right child of node at index `i`
        int left = LEFT(i);
        int right = RIGHT(i);

        int largest = i;

        // compare `A[i]` with its left and right child
        // and find the largest value
        if (left < size() && A[left].value > A[i].value) {
            largest = left;
        }

        if (right < size() && A[right].value > A[largest].value) {
            largest = right;
        }

        // swap with a child having greater value and
        // call heapify-down on the child
        if (largest != i)
        {
            swap(A[i], A[largest]);
            heapify_down(largest);
        }
    }

    // Recursive heapify-up algorithm
    void heapify_up(int i)
    {
        // check if the node at index `i` and its parent violate the heap property
        if (i && A[PARENT(i)].value < A[i].value)
        {
            // swap the two if heap property is violated
            swap(A[i], A[PARENT(i)]);

            // call heapify-up on the parent
            heapify_up(PARENT(i));
        }
    }

public:
    PriorityQueue(){
        this->s=0;
    }
    void clean(){
        A.clear();
        s=0;
    }

    bool hasKey(const int key) {
        return pos.find(key) != pos.end();
    }

    // return size of the heap
    unsigned int size() {
        return s;
    }

    // Function to check if the heap is empty or not
    bool empty() {
        return size() == 0;
    }

    // insert key into the heap
    void push(int key, int value)
    {
        s++;
        Node n;
        pos[key] = s-1;
        n.value=value;
        n.key=key;
        // insert a new element at the end of the vector
        A.push_back(n);

        // get element index and call heapify-up procedure
        int index = size() - 1;
        heapify_up(index);
    }

    // Function to remove an element with the highest priority (present at the root)
    void pop()
    {
        // if the heap has no elements, throw an exception
        if (size() == 0)
        {
            throw out_of_range("Vector<X>::at() : "
                               "index is out of range(Heap underflow)");
        }

        // replace the root of the heap with the last element
        // of the vector
        A[0] = A.back();
        pos.erase(A.back().key);
        A.pop_back();
        s--;
        // call heapify-down on the root node
        heapify_down(0);

    }
    void increaseKey(const int key, const int value) {
        if (!hasKey(key)) return; // key does not exist, do nothing
        int i = pos[key];
        if (value < A[i].value) return; // value would increase, do nothing
        A[i].value = value;
        heapify_up(i);
    }


    // Function to return an element with the highest priority (present at the root)
    int top()
    {
        // if the heap has no elements, throw an exception
        if (size() == 0)
        {
            throw out_of_range("Vector<X>::at() : "
                               "index is out of range(Heap underflow)");
        }

        // otherwise, return the top (first) element
        return A.at(0).key;        // or return A[0];
    }
};
#endif //DA_PROJ2_MAXHEAP_H