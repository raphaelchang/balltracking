/*
 * DisjointSet.h
 *
 *  Created on: Jan 2, 2014
 *      Author: raphael
 */

#ifndef DISJOINTSET_H_
#define DISJOINTSET_H_

#include <vector>

struct node {
	node *parent;
	int i, rank;
};

class DisjointSet {
public:
	DisjointSet();
	~DisjointSet();

	node* MakeSet(int i);
	node* Find(node* a);
	void Union(node* a0, node* a1);

	int ElementCount();
	int SetCount();

	int Reduce();
	void Reset();

private:
	std::vector<node*> nodes;
	unsigned int elements;
	int sets;
};

#endif /* DISJOINTSET_H_ */
