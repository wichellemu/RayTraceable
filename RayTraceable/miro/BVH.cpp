#include "BVH.h"
#include "Ray.h"
#include "Console.h"
#include "Lambert.h"

BVH::~BVH() {
    if(root) delete root;
}

void BVH::build(Objects * objs)
{
    printf("Building BVH...\n");
    // construct the bounding volume hierarchy
    m_objects = objs;
    root = new BVHNode();
    root->setObjects(objs);
    //root->setAxis();
    root->split(BVHNode::NUM_DIV * 10);
    printf("Done building BVH!\n");
    printf("Number of Nodes: %d\n", BVHNode::numNodes);
    printf("Number of Leaves: %d\n", BVHNode::numLeaf);
}


bool BVH::intersect(HitInfo& minHit, const Ray& ray, float tMin, float tMax, bool shadow) const
{
    BVHNode::rayTotal++;
    bool hit = false;
    HitInfo tempMinHit;
    minHit.t = MIRO_TMAX;

    return root->intersect(minHit, ray, tMin, tMax, shadow);
    /*
    // Here you would need to traverse the BVH to perform ray-intersection
    // acceleration. For now we just intersect every object.
    for (size_t i = 0; i < m_objects->size(); ++i)
    { 
    if ((*m_objects)[i]->intersect(tempMinHit, ray, tMin, tMax))
    {
        hit = true;
        if (tempMinHit.t < minHit.t)
        minHit = tempMinHit;
    }
    }

    return hit;
    */
}
