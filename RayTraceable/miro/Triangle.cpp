#include "Triangle.h"
#include "TriangleMesh.h"
#include "Ray.h"


extern float min( float a, float b, float c );
extern float max( float a, float b, float c );


Triangle::Triangle(TriangleMesh * m, unsigned int i) :
  m_mesh(m), m_index(i)
{

}


Triangle::~Triangle()
{

}


void Triangle::renderGL()
{
    TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
    const Vector3 & v0 = m_mesh->vertices()[ti3.x]; //vertex a of triangle
    const Vector3 & v1 = m_mesh->vertices()[ti3.y]; //vertex b of triangle
    const Vector3 & v2 = m_mesh->vertices()[ti3.z]; //vertex c of triangle

    glBegin(GL_TRIANGLES);
    glVertex3f(v0.x, v0.y, v0.z);
    glVertex3f(v1.x, v1.y, v1.z);
    glVertex3f(v2.x, v2.y, v2.z);
    glEnd();
}



bool Triangle::intersect(HitInfo& result, const Ray& r,float tMin, float tMax, bool shadow)
{
    if( shadow && !m_material->shadowable() ) {
    //fprintf(stderr, "Shadow material\n");
    return false;
    }
    // Get vertices
    TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
    const Vector3 & va = m_mesh->vertices()[ti3.x]; //vertex a of triangle
    const Vector3 & vb = m_mesh->vertices()[ti3.y]; //vertex b of triangle
    const Vector3 & vc = m_mesh->vertices()[ti3.z]; //vertex c of triangle

    TriangleMesh::TupleI3 tin3 = m_mesh->nIndices()[m_index];
    const Vector3 & na = m_mesh->normals()[tin3.x]; //vertex a of triangle
    const Vector3 & nb = m_mesh->normals()[tin3.y]; //vertex b of triangle
    const Vector3 & nc = m_mesh->normals()[tin3.z]; //vertex c of triangle

    // Set up the vectors for Barycentric triangle intersection
    Vector3 d = -r.d;
    Vector3 ba = vb-va;
    Vector3 ca = vc-va;
    Vector3 oa = r.o - va;

    // Calculate determinants
    float det = dot(d, cross(ba,ca));
    if( det == 0 ) 
    {
        return false;    // Check for divide by 0
    }

    float det_t = dot(oa, cross(ba,ca));
    float det_alpha = dot(d, cross(oa,ca));
    float det_beta = dot(d, cross(ba,oa));

    // Calculate variables
    float t = det_t / det;
    float alpha = det_alpha / det;
    float beta = det_beta / det;

    result.t = t;

    // Test Barycentric coordinates for miss
    if( alpha < 0 || beta < 0 || alpha + beta > 1 || t < tMin || t > tMax)
    {
        return false;
    }
    
    result.P = va + alpha*(ba) + beta*(ca);
    result.N = (1-alpha-beta)*na + alpha*nb + beta*nc;
    result.material = m_material;

    return true;
}

Vector3 Triangle::getCentroid() {
    // Get vertices
    TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
    const Vector3 & va = m_mesh->vertices()[ti3.x]; //vertex a of triangle
    const Vector3 & vb = m_mesh->vertices()[ti3.y]; //vertex b of triangle
    const Vector3 & vc = m_mesh->vertices()[ti3.z]; //vertex c of triangle
    return (va + vb + vc) / 3;
}

Vector3 Triangle::getMin() {
    TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
    const Vector3 & va = m_mesh->vertices()[ti3.x]; //vertex a of triangle
    const Vector3 & vb = m_mesh->vertices()[ti3.y]; //vertex b of triangle
    const Vector3 & vc = m_mesh->vertices()[ti3.z]; //vertex c of triangle
    return Vector3(min(va.x,vb.x,vc.x),
                    min(va.y,vb.y,vc.y),
                    min(va.z,vb.z,vc.z));
}

Vector3 Triangle::getMax() {
    TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
    const Vector3 & va = m_mesh->vertices()[ti3.x]; //vertex a of triangle
    const Vector3 & vb = m_mesh->vertices()[ti3.y]; //vertex b of triangle
    const Vector3 & vc = m_mesh->vertices()[ti3.z]; //vertex c of triangle
    return Vector3(max(va.x,vb.x,vc.x),
                    max(va.y,vb.y,vc.y),
                    max(va.z,vb.z,vc.z));
}

// Randomly sample the triangle using Barycentric coordinates without doing 
// rejection sampling
Vector3 Triangle::sample(Vector3 * normal) {
    // Get vertices
    TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
    const Vector3 & va = m_mesh->vertices()[ti3.x]; //vertex a of triangle
    const Vector3 & vb = m_mesh->vertices()[ti3.y]; //vertex b of triangle
    const Vector3 & vc = m_mesh->vertices()[ti3.z]; //vertex c of triangle
    Vector3 ba = vb-va;
    Vector3 ca = vc-va;

    float beta = ((float)rand()) / RAND_MAX;
    float gamma = ((float)rand()) / RAND_MAX;
    // Test if not triangle.  If so, transform into triangle
    if( beta + gamma > 1 ) {
        beta = 1-beta;
        gamma = 1-gamma;
    }

    // Set normal
    if( normal != nullptr ) {
        TriangleMesh::TupleI3 ti3 = m_mesh->nIndices()[m_index];
        const Vector3 & na = m_mesh->normals()[ti3.x]; //vertex a of triangle
        const Vector3 & nb = m_mesh->normals()[ti3.y]; //vertex b of triangle
        const Vector3 & nc = m_mesh->normals()[ti3.z]; //vertex c of triangle
        Vector3 n = (1-beta-gamma)*na + beta*nb + gamma*nc;
        normal->x = n.x;
        normal->y = n.y;
        normal->z = n.z;
    }

    return va + beta*ba + gamma*ca;
}