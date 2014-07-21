/*
 * TetMesh.h
 *
 *  Created on: May 6, 2014
 *      Author: pourya
 */

#ifndef CELLULARMESH_H
#define CELLULARMESH_H

#include "graphics/SGNode.h"
#include "CellularMeshTypes.h"
#include <functional>

/*!Stories:
 * 1. Iterate over edges
 * 2. Setup by adding vertices and tet elements
 * 3. Store rest positions for all vertices
 * 4. Add vertices and split edges using new vertices
 * 5. Iterate over elements
 *
*/
using namespace PS;
using namespace PS::SG;
using namespace std;


namespace PS {
namespace MESH {

//To track changes made to the topology of the mesh the following events are generated:
// 1. Node added/Removed
// 2. Edge added/Removed
// 3. Face added/Removed
// 4. Element added/Removed



//template <typename T>
class CellMesh : public SGNode {
public:
	static const U32 INVALID_INDEX = -1;
	enum TopologyEvent {teAdded, teRemoved};

	enum ErrorCodes {
		err_op_failed = -1,
		err_elem_not_found = -2,
		err_face_not_found = -3,
		err_edge_not_found = -4,
		err_node_not_found = -5,
	};


	typedef std::function<void(NODE, U32 handle, TopologyEvent event)> OnNodeEvent;
	typedef std::function<void(EDGE, U32 handle, TopologyEvent event)> OnEdgeEvent;
	typedef std::function<void(FACE, U32 handle, TopologyEvent event)> OnFaceEvent;
	typedef std::function<void(CELL, U32 handle, TopologyEvent event)> OnElementEvent;
public:
	CellMesh();
	CellMesh(const CellMesh& other);
	CellMesh(U32 ctVertices, double* vertices, U32 ctElements, U32* elements);
	CellMesh(const vector<double>& vertices, const vector<U32>& elements);
	virtual ~CellMesh();

	//Create one tet
	static CellMesh* CreateOneTet();

	//Topology events
	void setOnNodeEventCallback(OnNodeEvent f);
	void setOnEdgeEventCallback(OnEdgeEvent f);
	void setOnFaceEventCallback(OnFaceEvent f);
	void setOnElemEventCallback(OnElementEvent f);

	//Build
	bool setup(const vector<double>& vertices, const vector<U32>& elements);
	bool setup(U32 ctVertices, const double* vertices, U32 ctElements, const U32* elements);
	void cleanup();
	void printInfo() const;

	double computeDeterminant(U32 idxNodes[4]) const;
	static double ComputeElementDeterminant(const vec3d v[4]);

	//Index control
	inline bool isElemIndex(U32 i) const { return (i < m_vElements.size());}
	inline bool isFaceIndex(U32 i) const { return (i < m_vFaces.size());}
	inline bool isHalfEdgeIndex(U32 i) const { return (i < m_vHalfEdges.size());}
	inline bool isEdgeIndex(U32 i) const { return (i < countEdges());}
	inline bool isNodeIndex(U32 i) const { return (i < m_vNodes.size());}

	//access
	bool getFaceNodes(U32 idxFace, U32 (&nodes)[3]) const;

	CELL& elemAt(U32 i);
	FACE& faceAt(U32 i);
	HEDGE& halfedgeAt(U32 i);
	NODE& nodeAt(U32 i);
	EDGE edgeAt(U32 i) const;

	const CELL& const_elemAt(U32 i) const;
	const FACE& const_faceAt(U32 i) const;
	const HEDGE& const_halfedgeAt(U32 i) const;
	const NODE& const_nodeAt(U32 i) const;

	inline U32 countElements() const { return m_vElements.size();}
	inline U32 countFaces() const {return m_vFaces.size();}
	inline U32 countHalfEdges() const {return m_vHalfEdges.size();}
	inline U32 countEdges() const {return m_vHalfEdges.size() / 2;}
	inline U32 countNodes() const {return m_vNodes.size();}

	//functions
	inline U32 next_hedge(U32 he) const { return m_vHalfEdges[he].next;}
	inline U32 prev_hedge(U32 he) const { return m_vHalfEdges[he].prev;}
	inline U32 opposite_hedge(U32 he) const { return m_vHalfEdges[he].opposite;}

	inline U32 vertex_from_hedge(U32 he) const { return m_vHalfEdges[he].from;}
	inline U32 vertex_to_hedge(U32 he) const {return m_vHalfEdges[he].to;}
	inline U32 halfedge_from_edge(U32 edge, U8 which) const { return edge * 2 + which;}
	inline U32 edge_from_halfedge(U32 he) const { return he / 2;}


	//algorithm
	//splits an edge e at parametric point t
	void displace(double * u);


	//topology modifiers

	//insertions
	bool insert_element(const CELL& e);
	bool insert_element(U32 nodes[4]);
	U32 insert_face(U32 nodes[3]);
	U32 insert_node(const NODE& n);


	//remove
	void remove_element(U32 i);
	void remove_face(U32 i);

	//erases all objects marked removed
	void garbage_collection();


	/*!
	 * cuts an edge completely. Two new nodes are created at the point of cut with no hedges between them.
	 */
	bool cut_edge(int idxEdge, double distance, U32* poutIndexNP0 = NULL, U32* poutIndexNP1 = NULL);


	//algorithmic functions useful for many computational geometry projects
	int getFirstRing(int idxNode, vector<U32>& ringNodes) const;
	int getIncomingHalfEdges(int idxNode, vector<U32>& incomingHE) const;
	int getOutgoingHalfEdges(int idxNode, vector<U32>& outgoingHE) const;

	//checking
	void setElemToShow(U32 elem = INVALID_INDEX);
	U32 getElemToShow() const {return m_elemToShow;}

	//serialize
	bool readVegaFormat(const AnsiStr& strFP);
	bool writeVegaFormat(const AnsiStr& strFP) const;


	//draw
	void draw();
	void drawElement(U32 i) const;

	//aabb
	AABB computeAABB();

	//test that all faces are mapped to a correct key
	bool tst_keys();
private:
	void init();
	inline bool insertHEdgeIndexToMap(U32 from, U32 to, U32 idxHE);
	inline bool removeHEdgeIndexFromMap(U32 from, U32 to);

	inline bool insertFaceIndexToMap(U32 nodes[3], U32 idxFace);
	inline bool removeFaceIndexFromMap(U32 nodes[3]);

	inline HEdgeKey computeHEdgeKey(U32 idxHEdge) const;
	inline FaceKey computeFaceKey(U32 idxFace) const;


	inline bool halfedge_exists(U32 from, U32 to) const;
	U32 halfedge_handle(U32 from, U32 to);

protected:
	U32 m_elemToShow;

	//topology events
	OnNodeEvent m_fOnNodeEvent;
	OnEdgeEvent m_fOnEdgeEvent;
	OnFaceEvent m_fOnFaceEvent;
	OnElementEvent m_fOnElementEvent;

	//containers
	vector<CELL> m_vElements;
	vector<FACE> m_vFaces;
	vector<HEDGE> m_vHalfEdges;
	vector<NODE> m_vNodes;

	//maps a facekey to the corresponding face handle
	std::map< FaceKey, U32 > m_mapFaces;

	//maps a half-edge from-to pair to the corresponding hedge handle
	std::map< HEdgeKey, U32 > m_mapHalfEdgesIndex;

	//iterators
	typedef std::map< HEdgeKey, U32 >::iterator MAPHEDGEINDEXITER;
	typedef std::map< HEdgeKey, U32 >::iterator MAPHEDGEINDEXCONSTITER;
	typedef std::map< FaceKey, U32 >::iterator MAPFACEITER;
};

}
}

#endif /* TETMESH_H_ */
