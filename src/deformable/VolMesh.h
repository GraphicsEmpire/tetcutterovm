/*
 * VolMesh.h
 *
 *  Created on: Jul 21, 2014
 *      Author: pourya
 */

#ifndef VOLMESH_H_
#define VOLMESH_H_

#include "graphics/SGNode.h"
#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

using namespace PS::SG;

namespace PS {
namespace MESH {

typedef OpenVolumeMesh::Geometry::Vec3d         Vec3d;
typedef OpenVolumeMesh::GeometryKernel<Vec3d>   VolMesh;


}
}



#endif /* VOLMESH_H_ */
