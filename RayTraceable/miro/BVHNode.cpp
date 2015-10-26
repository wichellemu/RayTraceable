#include "BVHNode.h"
#include "Ray.h"

unsigned long long BVHNode::numLeaf = 0;
unsigned long long BVHNode::numNodes = 0;
unsigned long long BVHNode::rayBox = 0;
unsigned long long BVHNode::rayBox2 = 0;
unsigned long long BVHNode::rayTri = 0;
unsigned long long BVHNode::rayTotal = 0;


inline float min( float a, float b, float c ) {
  if( a < b ) {
    if( a < c ) return a;
    return c;
  }
  if( b < c ) return b;
  return c;
}
inline float max( float a, float b, float c ) {
  if( a > b ) {
    if( a > c ) return a;
    return c;
  }
  if( b > c ) return b;
  return c;
}
inline void swap(float & a, float & b) {
  float tmp = a;
  a = b;
  b = tmp;
}
BVHNode::BVHNode() : plane_position(0), m_axis(AXIS_ERROR), is_leaf(false),
          left(NULL), right(NULL), m_boxmin(Vector3()), m_boxmax(Vector3()) {
  objects = new vector<Object *>();
  m_minCost = numeric_limits<float>::infinity();
  indeedleaf = false;
}

BVHNode::~BVHNode() {
  objects->clear();
  delete objects;
  if( left )
    delete left;
  if( right )
    delete right;
}

bool BVHNode::hit(const Ray& ray) {
	rayBox2++;
  float txmin, txmax, tymin, tymax, tzmin, tzmax;
  txmin = (m_boxmin.x - ray.o.x) / ray.d.x;
  txmax = (m_boxmax.x - ray.o.x) / ray.d.x;
  if( txmin > txmax ) swap(txmin, txmax);

  tymin = (m_boxmin.y - ray.o.y) / ray.d.y;
  tymax = (m_boxmax.y - ray.o.y) / ray.d.y;
  if( tymin > tymax ) swap(tymin, tymax);

  tzmin = (m_boxmin.z - ray.o.z) / ray.d.z;
  tzmax = (m_boxmax.z - ray.o.z) / ray.d.z;
  if( tzmin > tzmax ) swap(tzmin, tzmax);

  float tmin = max(txmin, tymin, tzmin);
  float tmax = min(txmax, tymax, tzmax);

  if( tmin > tmax || tmax < 0 ) {
    return false;
  }
  return true;
}

bool BVHNode::intersect(HitInfo& result, const Ray& ray,
        float tMin, float tMax, bool shadow) {
if (objects==nullptr)
{
		return 0;
}
  bool hit = false;
  if( this->hit(ray) ) {
    if( is_leaf ) {
        result.t = numeric_limits<float>::infinity();
      for( Object * obj : *objects ) {
		  rayTri++;
		  HitInfo tmpHitInfo;
        if( obj->intersect(tmpHitInfo, ray, tMin, tMax, shadow) ) {
          if( tmpHitInfo.t < result.t ) {
			
            hit = true;
            result = tmpHitInfo;
          }
          
        }
      }
    }
    else {
		if (left != nullptr)
		{
			
			hit = left->intersect(result, ray, tMin, tMax, shadow);
		}
      if( hit ) {
		  rayBox++;
        HitInfo tmpHitInfo;
        if( right != nullptr && right->intersect(tmpHitInfo, ray, tMin, result.t, shadow) && tmpHitInfo.t < result.t ) {
          result = tmpHitInfo;
		  rayBox++;
        }
      }
	  else if (right != nullptr)
	  {

		  hit = right->intersect(result, ray, tMin, tMax, shadow);
		  if (hit)
		  {
			  rayBox++;
		  }
	  }
    }
  }
  return hit;
}


int BVHNode::getDividAxis()
{
	Vector3 a = m_boxmax - m_boxmin;
	a.x = abs(a.x);
	a.y = abs(a.y);
	a.z = abs(a.z);
	if (a.x >= a.y && a.x >= a.z)
	{
		return AXIS_X;
	}
	else if (a.y >= a.z && a.y >= a.x)
	{
		return AXIS_Y;
	}
	else if (a.z >= a.x && a.z >= a.y)
	{
		return AXIS_Z;
	}
	else
	{
		fprintf(stderr, "Error has occured while picking an axis");
		return AXIS_ERROR;
	}
}

void BVHNode::setAxis() {
  m_axis = getDividAxis();
}

void BVHNode::setObjects(Objects * objs) {
  for( Object * obj : *objs ) {
    addObject(obj);
  }
}

void BVHNode::addObject(Object * o) {
  objects->push_back(o);
  Vector3 min = o->getMin();
  Vector3 max = o->getMax();
  if( objects->size() == 1 ) {    // First object
    for(int axis = AXIS_X; axis <= AXIS_Z; axis++) {
      m_boxmin[axis] = min[axis];
      m_boxmax[axis] = max[axis];
    }
  }
  else {
    for(int axis = AXIS_X; axis <= AXIS_Z; axis++) {
      if( min[axis] < m_boxmin[axis] ) m_boxmin[axis] = min[axis];
      if( max[axis] > m_boxmax[axis] ) m_boxmax[axis] = max[axis];
    }
  }
}

void BVHNode::split(int numSplit) {
  numNodes++;
  if( objects->size() < MAX_OBJS || indeedleaf) {
    numLeaf++;
    setLeaf(true);
    return;
  }


  for(int axis = AXIS_X; axis <= AXIS_Z; axis++) {
    splitAxis(axis, numSplit);
  }

  objects->clear();
  // Recursively split
 // printf("Right: %d\n", right->objects->size());
  if (right->objects->size() == objects->size())
  {
	  right->setLeaf(true);
	  return;
  }
 
  right->split();
 // printf("Left: %d\n\n", left->objects->size());
  if (left->objects->size() == objects->size())
  {
	  left->setLeaf(true);
	  return;
  }

  left->split();
}

// Recursively do this
void BVHNode::splitAxis(int axis, int numSplit) {
 if( axis >= AXIS_ERROR ) {
    fprintf(stderr, "Bad axis in split method\n");
  }
  float bMax = m_boxmax[axis], bMin = m_boxmin[axis];

  vector<struct bin> bins = vector<struct bin>(numSplit);
  

  float deltaPos = (bMax - bMin) / numSplit;
  
  // Initialize min/max of bins
  for(int j = 0; j < numSplit; ++j) {
    bins[j].divMin = bMin + j*deltaPos;
    bins[j].divMax = bMin + (j+1)*deltaPos;
  }

  // Loops through objects and place in bins
  for( Object * obj : *objects ) {
    Vector3 centroid = obj->getCentroid();
    float testCoord = centroid[axis];

    bool foundMin = false;
    // Figure out which bin to put in
    for(int i = 0; i < numSplit; i++ ) {
      if( testCoord <= bins[i].divMax ) {
        bins[i].objects->push_back(obj);
        break;
      }
    }
  }

 

  // Find where to split
  float minCost = numeric_limits<float>::infinity();
  int lastLeft;
  for(int i = 0; i < numSplit-1; ++i) {
    float cost = getCost(axis, bins, i);
    if( cost <= minCost ) {
      minCost = cost;
      lastLeft = i;
    }
  } // End for loop
  
  if( minCost <= m_minCost ) {
    m_axis = axis;
    m_minCost = minCost;
    plane_position = bins[lastLeft].divMax;

    if( left ) delete left;
    if( right ) delete right;
    left = new BVHNode();
    right = new BVHNode();

    for(int i = 0; i <= lastLeft; ++i) {
      for( Object * obj : *bins[i].objects ) { 
        left->addObject(obj);
      }
    }
    for(int i = lastLeft+1; i < numSplit; ++i) {
      for( Object * obj : *bins[i].objects ) { 
        right->addObject(obj);
      }
    }
	
  }
  
}

inline float SA(float a, float b, float c) {
  return 2*a*b + 2*(a+b)*c;
}

// Last left includes this index in the left child box
float BVHNode::getCost(int axis, vector<struct bin> & bins, int lastLeft) {
  if( axis == AXIS_ERROR )
    fprintf(stderr, "Bad axis in cost function\n");

  float aDist = m_boxmax[(axis+1)%3] - m_boxmin[(axis+1)%3];
  float bDist = m_boxmax[(axis+2)%3] - m_boxmin[(axis+2)%3];
  float bMin = m_boxmin[axis], bMax = m_boxmax[axis], bSplit;

  bSplit = bins[lastLeft].divMax;
  int numLeft = 0, numRight = 0;

  // Count how many are in the left and right boxes
  int size = bins.size();
  for(int i = 0; i < size; ++i) {
    if( i <= lastLeft )
      numLeft += bins[i].objects->size();
    else
      numRight += bins[i].objects->size();
  }
  if( numLeft == 0 || numRight == 0 )
    return SA(aDist, bDist, bMax-bMin) * (numLeft + numRight);
  return SA(aDist, bDist, (bSplit-bMin))*numLeft + 
          SA(aDist, bDist, (bMax-bSplit))*numRight;
}
