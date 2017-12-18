
#ifndef _BALL_H_
#define _BALL_H_

// includes
#include "../Math/Vector3D.h"
#include "Physics.h"


// forwards

// Definicion de la clase
class CBall:public ICircleSegmentContactReport
{

public:

	// Constructor / destructor
	CBall( int resX, int resY, const CVector3D &pos );
	~CBall( void );

	// Pintado
	void Render( void );

	virtual void onContact(const CVector3D &colNormal);
private:

	// Radio de circulo
	float m_radius;

	// Posicion
	CVector3D m_pos;

	// Resolucion de pantalla
	int m_resX, m_resY;

	// Puntero al actor de fisicas
	CPhysics::PhysicActor *m_physicsActor;
};

#endif