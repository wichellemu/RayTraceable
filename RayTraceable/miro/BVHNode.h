#ifndef BVHNode_H
#define BVHNode_H

#include "Object.h"
#include <vector>
#include <cstdlib>
#include <cmath>

using namespace std;

class BVHNode {
public:
  bool indeedleaf;
  static const int NUM_DIV = 5;
  static const int AXIS_X = 0;
  static const int AXIS_Y = 1;
  static const int AXIS_Z = 2;
  static const int AXIS_ERROR = 3;
  static const int MAX_OBJS = 8;
  static unsigned long long numLeaf, numNodes, rayBox, rayTri, rayBox2, rayTotal;

  BVHNode();
  ~BVHNode();
  
  static void resetStats() { rayTotal = 0;  numLeaf = 0; numNodes = 0; rayBox = rayTri = rayBox2 = 0; }

  bool hit( const Ray& ray );
  bool intersect(HitInfo& result, const Ray& ray,
        float tMin = 0.0f, float tMax = MIRO_TMAX, bool shadow = false);

  bool isLeaf() { return is_leaf; }
  void setLeaf(bool leaf) { is_leaf = leaf; }
  void setBoxMin(Vector3 boxmin) { m_boxmin = boxmin; }
  void setBoxMax(Vector3 boxmax) { m_boxmin = boxmax; }
  void addObject(Object * o);
  void setObjects(Objects * objs);

  void split(int numSplit = NUM_DIV);
  int getDividAxis();
  void setAxis();

private:
  struct bin {        // Switch if needed access outside BVHNode
    vector<Object*> * objects;
    float divMin, divMax;
    bin() {
      objects = new vector<Object*>();
    }
    ~bin() {
      objects->clear();
      //delete objects;
    }
  };

  float plane_position;
  float m_minCost;
  int m_axis;
  bool is_leaf;
  BVHNode * left, *right;
  Objects * objects;     // "Vector of Triangles" - Krishna
  Vector3 m_boxmin, m_boxmax;


  float getCost(int axis, vector<struct bin> & bins, int lastLeft);
  void splitAxis(int axis, int numSplit = NUM_DIV);
};


#endif