/*
 * AvatarScalpel.h
 *
 *  Created on: Apr 6, 2014
 *      Author: pourya
 */

#ifndef AVATARSCALPEL_H_
#define AVATARSCALPEL_H_

#include "graphics/SGMesh.h"
#include "graphics/Gizmo.h"
#include "CuttableMesh.h"

using namespace PS;
using namespace PS::SG;

#define MAX_SCALPEL_TRAJECTORY_ANGLE  60.0
#define MAX_SCALPEL_TRAJECTORY_NODES 1024

/*!
 * Synopsis: Haptics Avatar guide
 */
class AvatarScalpel : public SGMesh, public IGizmoListener {
public:
	AvatarScalpel();
	AvatarScalpel(CuttableMesh* tissue);
	virtual ~AvatarScalpel();

	void init();
	void draw();


	//Tool
	bool isActive() const {return m_isToolActive;}
	void clearCutContext();
	void setTissue(CuttableMesh* tissue);


	//From Gizmo Manager
	void mousePress(int button, int state, int x, int y);
	void onTranslate(const vec3f& delta, const vec3f& pos);

public:

protected:
	AABB m_aabbCurrent;
	CuttableMesh* m_lpTissue;

	//Outline mesh for easier view
	SGMesh m_outline;

	//blade edges
	vec3f m_edgeref0;
	vec3f m_edgeref1;

	//cut info
	vector<vec3d> m_vCuttingPathEdge0;
	vector<vec3d> m_vCuttingPathEdge1;
	vec3d m_sweptQuad[4];
	bool m_isSweptQuadValid;
	bool m_isToolActive;
};



#endif /* AVATARSCALPEL_H_ */
