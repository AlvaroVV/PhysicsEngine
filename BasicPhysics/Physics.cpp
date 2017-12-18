
#include "Physics.h"
#include <stdlib.h>
#include <math.h>

__inline float Randf( float min, float max )
{
	return (float)( ((rand()&32767)*(1.0/32767.0))*(max-min) + min );
}

/*----------------------------------------------------------------------------------------------------------------------*/

CPhysics::CPhysics( const CVector3D &gravity )
{
	m_gravityForce = gravity;
	m_defaultMaterial.restitution = 0.7f;
	m_numStatic = 0;
	m_numDynamic = 0;
	m_MaxTimeToSleep = 3;
	m_MaxDistanceToSleep = 0.001f;
}

/*----------------------------------------------------------------------------------------------------------------------*/

CPhysics::~CPhysics( void )
{
	for( unsigned int i=0; i<m_numStatic; ++i ) {
		delete [] m_staticActors[i]->segments;
		delete m_staticActors[i];
	}
	for( unsigned int i=0; i<m_numDynamic; ++i ) {
		delete m_dynamicActors[i];
	}
}

/*----------------------------------------------------------------------------------------------------------------------*/

void CPhysics::Update( float timeDiff )
{
	for( unsigned int i=0; i<m_numDynamic; ++i ) {
		if(!m_dynamicActors[i]->actorInfo.sleeping)
			UpdateDymanicPos( *m_dynamicActors[i], timeDiff );
	}
}

/*----------------------------------------------------------------------------------------------------------------------*/

void CPhysics::UpdateDymanicPos( sDynamicGeometryCircle &geom, float timeDiff )
{

	CVector3D prePos = CVector3D(geom.actorInfo.pos);
	geom.actorInfo.vel += m_gravityForce * timeDiff;
	geom.actorInfo.pos += geom.actorInfo.vel * timeDiff;

	float numSegments = 0;
	sSegment* segments = nullptr;
	CVector3D col;
	CVector3D normal;

	//Comprobamos colisiones Circulo-Circulo
	for (unsigned int i = 0; i < m_numDynamic; i++)
	{
		if(geom.actorInfo.pos != m_dynamicActors[i]->actorInfo.pos)
			if (CheckCircleToCircleCollision(geom.actorInfo.pos, geom.radius, m_dynamicActors[i]->actorInfo.pos, m_dynamicActors[i]->radius, normal))
			{
				//La velocidad resultante del choque se calcula como la normal de la colisión por la magnitud
				//TODO: No estamos teniendo en cuenta la posible velocidad del circulo con el que choca.
				geom.actorInfo.vel = normal* (geom.actorInfo.vel.GetMagnitude() * geom.actorInfo.mat.restitution);
				geom.actorInfo.pos = prePos;
				//calculamos la velocidad nueva del circulo con el que chocamos y le aplicamos un descenso del 80%.
				m_dynamicActors[i]->actorInfo.vel = (normal * (geom.actorInfo.vel.GetMagnitude() * 0.8f));
				m_dynamicActors[i]->actorInfo.vel.Negate();
				
				//Tenemos un margen para poner a dormir elementos con MaxDistanceToSleep.
				if (m_dynamicActors[i]->actorInfo.vel.GetMagnitude() > m_MaxDistanceToSleep)
				{
					m_dynamicActors[i]->actorInfo.timeUnderThreshold = 0.0f;
					m_dynamicActors[i]->actorInfo.sleeping = false;
				}
				
			}
	}

	for (unsigned int i = 0; i < m_numStatic; i++)
	{
		numSegments = m_staticActors[i]->numSegments;
		segments = m_staticActors[i]->segments;
		for (unsigned int j = 0; j < numSegments; j++)
		{
			if (CheckCircleSegmentCollision(geom.actorInfo.pos, geom.radius, segments[j], col, normal))
			{
				
				geom.actorInfo.pos = col;

				CVector3D incidente = geom.actorInfo.vel;
				incidente.Negate();
				float angle = atan2(incidente.x, incidente.y) - atan2(normal.x, normal.y);
				angle *= 2;
				float x2 = incidente.x * cos(angle) - incidente.y * sin(angle);
				float y2 = incidente.x * sin(angle) + incidente.y * cos(angle);
				geom.actorInfo.vel = CVector3D(x2, y2, 0.0f) * (sqrt(pow(geom.actorInfo.vel.x, 2) + pow(geom.actorInfo.vel.y, 2)) * geom.actorInfo.mat.restitution);
				if(geom.actorInfo.report)
					geom.actorInfo.report->onContact(normal);
			}

			sSegment newSegment;
			newSegment.p[0] = prePos;
			newSegment.p[1] = prePos + geom.actorInfo.vel * timeDiff;
			CVector3D posCollision;
			if (CheckSegmentToSegmentCollision(newSegment, segments[j], posCollision))
			{
				geom.actorInfo.pos = prePos;
				//geom.actorInfo.vel = CVector3D(0.0f, 0.0f, 0.0f);
			}
		}
	}


	if ((geom.actorInfo.pos - prePos).GetMagnitude() < m_MaxDistanceToSleep)
	{
		geom.actorInfo.timeUnderThreshold += timeDiff;
		if (geom.actorInfo.timeUnderThreshold >= m_MaxTimeToSleep)
		{
			geom.actorInfo.sleeping = true;
			geom.actorInfo.vel = CVector3D(0, 0, 0);
		}
	}
	else
		geom.actorInfo.timeUnderThreshold = 0.0f;

	


}

/*----------------------------------------------------------------------------------------------------------------------*/

bool CPhysics::CheckCircleSegmentCollision( const CVector3D &circlePos, float circleRadius, const sSegment &segment, CVector3D &col, CVector3D &normal )
{
	CVector3D segV = segment.p[1] - segment.p[0];
	CVector3D ptV = circlePos - segment.p[0];

	// Calculamos el punto mas cercano desde el centro del circulo al segmento
	// proyectamos ptV sobre segV
	float projVMod = ptV * ( segV / segV.GetMagnitude() );

	// projVMod es la magnitud del vector proyeccion
	CVector3D closestPoint;
	if( projVMod < 0 ) {
		closestPoint = segment.p[0];
	}
	else if( projVMod > segV.GetMagnitude() ) {
		closestPoint = segment.p[1];
	}
	else {
		// relativo a p[0]
		CVector3D projV = ( segV / segV.GetMagnitude() ) * projVMod;

		// relativo a coordenadas de mundo
		closestPoint = segment.p[0] + projV;
	}
	
	// Vector al centro del circulo
	CVector3D distV = circlePos - closestPoint;

	// Si la magnitud es menor que el radio, hay colision
	float distVMod = distV.GetMagnitude();
	if( distVMod < circleRadius ) {
		normal = distV;
		normal.Normalize();
		col = circlePos + ( normal * ( circleRadius - distVMod ) );
		return true;
	}
	return false;
}

bool CPhysics::CheckSegmentToSegmentCollision(const sSegment &s0, const sSegment &s1, CVector3D &resul)
{
	float x1_ = s0.p[0].x;
	float x2_ = s0.p[1].x;
	float y1_ = s0.p[0].y;
	float y2_ = s0.p[1].y;
	float x3_ = s1.p[0].x;
	float x4_ = s1.p[1].x;
	float y3_ = s1.p[0].y;
	float y4_ = s1.p[1].y;
	float r, s, d;

	// Si no son paralelas...
	if ((y2_ - y1_) / (x2_ - x1_) != (y4_ - y3_) / (x4_ - x3_))
	{
		d = (((x2_ - x1_) * (y4_ - y3_)) - (y2_ - y1_) * (x4_ - x3_));
		if (d != 0)
		{
			r = (((y1_ - y3_) * (x4_ - x3_)) - (x1_ - x3_) * (y4_ - y3_)) / d;
			s = (((y1_ - y3_) * (x2_ - x1_)) - (x1_ - x3_) * (y2_ - y1_)) / d;
			if (r >= 0 && r <= 1)
			{
				if (s >= 0 && s <= 1)
				{
					resul.x = x1_ + r * (x2_ - x1_);
					resul.y = y1_ + r * (y2_ - y1_);
					return true;
				}
			}
		}
	}
	return false;
}

bool CPhysics::CheckCircleToCircleCollision(const CVector3D &circle1Pos, float radius1, const CVector3D &circle2Pos, float radius2, CVector3D &normal)
{
	//Cogemos el vector distancia entre circulo1 y circulo2.
	CVector3D vectorD = CVector3D(circle2Pos.x - circle1Pos.x, circle2Pos.y - circle1Pos.y, 0.0f);
	float magnitude = vectorD.GetMagnitude();
	
	//Si la magnitud del vector distancia es menor que la suma de los dos radios, quiere decir que están colisionando.
	if (magnitude <= radius1 + radius2)
	{
		normal.x = vectorD.x;
		normal.y = vectorD.y;
		normal.Normalize();
		//Debido a la foram de calcular el vector director, tenemos que negarlo para calcular la normal de la colisión.
		normal.Negate();
		return true;
	}
	else
		return false;
}

/*----------------------------------------------------------------------------------------------------------------------*/

void CPhysics::Render( void )
{
}

/*----------------------------------------------------------------------------------------------------------------------*/

CPhysics::PhysicActor *CPhysics::AddDynamicActor( const CVector3D &pos, float radius )
{
	if( m_numDynamic < MAX_DYNAMICS ) {
		sDynamicGeometryCircle *geom = new sDynamicGeometryCircle;
		geom->actorInfo.pos = pos;
		geom->actorInfo.vel = CVector3D( 0.0f, 0.0f, 0.0f );
		geom->actorInfo.mat = m_defaultMaterial;
		geom->radius = radius;
		geom->actorInfo.report = NULL;
		m_dynamicActors[m_numDynamic++] = geom;
		return &geom->actorInfo;
	}
	return 0;
}

/*----------------------------------------------------------------------------------------------------------------------*/

void CPhysics::AddStaticActor( sSegment *segments, unsigned int numSegments )
{
	sStaticGeometrySegments *geom = new sStaticGeometrySegments;
	geom->numSegments = numSegments;
	geom->segments = new sSegment[numSegments];
	for( unsigned int i=0; i<numSegments; ++i ) {
		geom->segments[i] = segments[i];
	}
	m_staticActors[m_numStatic++] = geom;
}

/*----------------------------------------------------------------------------------------------------------------------*/
