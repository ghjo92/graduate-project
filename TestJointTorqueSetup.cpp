//test addJointTorque
#include "TestJointTorqueSetup.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
							
#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../Utils/b3ResourcePath.h"

static btScalar radius(0.3);
static btScalar base_radius(0.6);

struct TestJointTorqueSetup : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;

    bool m_once;
public:

    TestJointTorqueSetup(struct GUIHelperInterface* helper);
    virtual ~TestJointTorqueSetup();

    virtual void initPhysics();

    virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 8;
		float pitch = 270;
		float yaw = 21;
		float targetPos[3]={-1.34,3.4,-0.44};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}


};

TestJointTorqueSetup::TestJointTorqueSetup(struct GUIHelperInterface* helper)
:CommonMultiBodyBase(helper),
m_once(true)
{
}

TestJointTorqueSetup::~TestJointTorqueSetup()
{

}

///this is a temporary global, until we determine if we need the option or not
extern  bool gJointFeedbackInWorldSpace;
extern bool gJointFeedbackInJointFrame;



void TestJointTorqueSetup::initPhysics()
{
    int upAxis = 1.0;		//1이랑 2... 평면에서 y축거리 차이?
	gJointFeedbackInWorldSpace = true;
	gJointFeedbackInJointFrame = true;

	m_guiHelper->setUpAxis(upAxis);

    btVector4 colors[4] =
    {
        btVector4(1,0,0,1),
        btVector4(0,1,0,1),
        btVector4(0,1,1,1),
        btVector4(1,1,0,1),
    };
    int curColor = 0;
    

	this->createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
        //btIDebugDraw::DBG_DrawConstraints
        +btIDebugDraw::DBG_DrawWireframe
        +btIDebugDraw::DBG_DrawContactPoints
        +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);	

    

    { //? 뭐에 중괄호가 걸린걸까.
        bool floating = false;	//유동적인
        bool damping = false;	//제동?감폭?
        bool gyro = false;		//회전
        int numLinks = 2;      //갯수 늘어나면 공이 추가됨.
        bool spherical = true;					//set it ot false -to use 1DoF hinges instead of 3DoF sphericals, 관절자유도.
        bool canSleep = false;
        bool selfCollide = false;
        btVector3 linkHalfExtents(0.05, 0.5, 0.05);			//막대기 x축. z축. y축
        btVector3 basePosition = btVector3(-2.f, 0.f, 0.f);


        //mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
        //init the base
        btVector3 baseInertiaDiag(0.f, 0.0f, 0.f);				//잘모르겠따
        float baseMass = 1.f;

        btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
		
        m_multiBody = pMultiBody;		//주석걸어도 실행됨.
        btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);			//2번째 박스 초기위치 각도.     yz평명 위쪽 ,xz평면,xy평면,yz평면
	//	baseOriQuat.setEulerZYX(-.25*SIMD_PI,0,-1.75*SIMD_PI);   
        pMultiBody->setBasePos(basePosition);
        pMultiBody->setWorldToBaseRot(baseOriQuat);
        btVector3 vel(0, 0, 0);					// 안쓰는듯. 밑에꺼 주석풀면 에러떠서 찾아봐야될듯. 아마 초기속도가 아닐까!
    //	pMultiBody->setBaseVel(vel);

        //init the links
        btVector3 hingeJointAxis(1, 0, 0);			//관절인듯?
        
        //y-axis assumed up , 관절 위치 연결.

        btVector3 parentComToCurrentCom(0, linkHalfExtents[1] * 2.f, 0);						//par body's COM to cur body's COM offset, 첫번째 관절과 박스 위치.
        btVector3 currentPivotToCurrentCom(0, linkHalfExtents[1], 0);							//cur body's COM to cur body's PIV offset,
        btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset

        //////
        btScalar q0 = 0.f * SIMD_PI/ 180.f;
        btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
        quat0.normalize();
        /////

        for(int i = 0; i < numLinks; ++i)
        {
			float linkMass = 1.f;
			//if (i==3 || i==2)
			//	linkMass= 1000;
			btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

			btCollisionShape* shape = 0;
			if (i==0)
			{
				shape = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));//
			} else
			{
				shape = new btSphereShape(radius);
			}
			shape->calculateLocalInertia(linkMass, linkInertiaDiag); //관성
			delete shape;


            if(spherical)   // 관절 자유도.
			{
                //pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, false);
		
				if (i==0)
				{
				/*pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, 
					btQuaternion(0.f, 0.f, 0.f, 1.f), 
					hingeJointAxis, 
					parentComToCurrentPivot, 
					currentPivotToCurrentCom, false);*/ //이게 회전이고 fixed가 고정인듯! 안움직이네

					//fixed 아래꺼로 실험해본거.
				/*pMultiBody->setupFixed(i, linkMass, linkInertiaDiag, i - 1,
					btQuaternion(0.f, 0.f, 0.f, 1.f),
					parentComToCurrentPivot,
					currentPivotToCurrentCom);*/
				//실험
				pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, false);


				} else
				{
					btVector3 parentComToCurrentCom(0, radius * 2.f, 0);						//par body's COM to cur body's COM offset, 두번째 관절과 작은 원.
					btVector3 currentPivotToCurrentCom(0, radius, 0);							//cur body's COM to cur body's PIV offset
					btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset


					/*pMultiBody->setupFixed(i, linkMass, linkInertiaDiag, i - 1, 
					btQuaternion(0.f, 0.f, 0.f, 1.f), 
					parentComToCurrentPivot, 
					currentPivotToCurrentCom);*/
					pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, false);
				}					
				//pMultiBody->setupFixed(i,linkMass,linkInertiaDiag,i-1,btQuaternion(0,0,0,1),parentComToCurrentPivot,currentPivotToCurrentCom,false);
		
			}
            else
			{
                //pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
                pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, false);

			}
        }

        pMultiBody->finalizeMultiDof();

		//for (int i=pMultiBody->getNumLinks()-1;i>=0;i--)//
		for (int i=0;i<pMultiBody->getNumLinks();i++)
		{
			btMultiBodyJointFeedback* fb = new btMultiBodyJointFeedback();
			pMultiBody->getLink(i).m_jointFeedback = fb;
			m_jointFeedbacks.push_back(fb);
			//break;
		}
        btMultiBodyDynamicsWorld* world = m_dynamicsWorld;

        ///
        world->addMultiBody(pMultiBody);
        btMultiBody* mbC = pMultiBody;
        mbC->setCanSleep(!canSleep);						//중간에 멈출 수 있게하는것.
        mbC->setHasSelfCollision(!selfCollide);				//베이스에 충돌 생성.
        mbC->setUseGyroTerm(!gyro);							//회전
        //
        
		if(!damping)		//damping 저항력.
        {
            mbC->setLinearDamping(0.f);
            mbC->setAngularDamping(0.f);
        }else
        {	mbC->setLinearDamping(0.1f);
            mbC->setAngularDamping(0.9f);
        }
        //
    	m_dynamicsWorld->setGravity(btVector3(0,10,0));			//중력 x,z,y축, 초기에 이것때문에 움직인다.

        //////////////////////////////////////////////
        if(0)//numLinks > 0) //1로바꾸면 실행안됨;
        {
            btScalar q0 = 45.f * SIMD_PI/ 180.f;
            if(!spherical)
			{
				mbC->setJointPosMultiDof(0, &q0);
			}
            else
            {
                btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
                quat0.normalize();
                mbC->setJointPosMultiDof(0, quat0);
            }
        }
        ///

        btAlignedObjectArray<btQuaternion> world_to_local;
        world_to_local.resize(pMultiBody->getNumLinks() + 1);

        btAlignedObjectArray<btVector3> local_origin;
        local_origin.resize(pMultiBody->getNumLinks() + 1);
        world_to_local[0] = pMultiBody->getWorldToBaseRot();			//rotates world vectors into base frame 랍니다.
        local_origin[0] = pMultiBody->getBasePos();
      //  double friction = 1;
        {

        //	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
//            btScalar quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};


            if (1)
            {
                btCollisionShape* shape = new btSphereShape(base_radius);
                m_guiHelper->createCollisionShapeGraphicsObject(shape);

                btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(pMultiBody, -1);
                col->setCollisionShape(shape);

                btTransform tr;
                tr.setIdentity();
//if we don't set the initial pose of the btCollisionObject, the simulator will do this 
				//when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider
               
                tr.setOrigin(local_origin[0]);
				btQuaternion orn(btVector3(0,0,1),0.25*3.1415926538);
				
                tr.setRotation(orn);
                col->setWorldTransform(tr);

				bool isDynamic = (baseMass > 0 && floating);
				short collisionFilterGroup = isDynamic? short(btBroadphaseProxy::DefaultFilter) : short(btBroadphaseProxy::StaticFilter);
				short collisionFilterMask = isDynamic? 	short(btBroadphaseProxy::AllFilter) : 	short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);


                world->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);//, 2,1+2);

                btVector3 color(0.0,0.0,1.0);
                m_guiHelper->createCollisionObjectGraphicsObject(col,color);

//                col->setFriction(friction);
                pMultiBody->setBaseCollider(col);

            }
        }


        for (int i=0; i < pMultiBody->getNumLinks(); ++i)
        {
            const int parent = pMultiBody->getParent(i);
            world_to_local[i+1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent+1];
            local_origin[i+1] = local_origin[parent+1] + (quatRotate(world_to_local[i+1].inverse() , pMultiBody->getRVector(i)));
        }


        for (int i=0; i < pMultiBody->getNumLinks(); ++i)
        {

            btVector3 posr = local_origin[i+1];
        //	float pos[4]={posr.x(),posr.y(),posr.z(),1};

            btScalar quat[4]={-world_to_local[i+1].x(),-world_to_local[i+1].y(),-world_to_local[i+1].z(),world_to_local[i+1].w()};
			btCollisionShape* shape =0;

			if (i==0)
			{
				shape = new btBoxShape(btVector3(linkHalfExtents[0],linkHalfExtents[1],linkHalfExtents[2]));//btSphereShape(linkHalfExtents[0]);
			} else
			{	
				shape = new btSphereShape(radius);
			}

            m_guiHelper->createCollisionShapeGraphicsObject(shape);
            btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

            col->setCollisionShape(shape);
            btTransform tr;
            tr.setIdentity();
            tr.setOrigin(posr);
            tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
            col->setWorldTransform(tr);
     //       col->setFriction(friction);
			bool isDynamic = 1;//(linkMass > 0); // 다이나믹이 아니면 초기회전시 y축 회전하고 selfcollison속성을 무시한다.
			short collisionFilterGroup = isDynamic? short(btBroadphaseProxy::DefaultFilter) : short(btBroadphaseProxy::StaticFilter);
			short collisionFilterMask = isDynamic? 	short(btBroadphaseProxy::AllFilter) : 	short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

			//if (i==0||i>numLinks-2)
			{
				world->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);//,2,1+2);
				   btVector4 color = colors[curColor];
			curColor++;
			curColor&=3;
            m_guiHelper->createCollisionObjectGraphicsObject(col,color);


            pMultiBody->getLink(i).m_collider=col;
			}
         
        }
    }

	btSerializer* s = new btDefaultSerializer;
	m_dynamicsWorld->serialize(s);
	b3ResourcePath p;
	char resourcePath[1024];
	if (p.findResourcePath("multibody.bullet",resourcePath,1024))
	{
		FILE* f = fopen(resourcePath,"wb");
		fwrite(s->getBufferPointer(),s->getCurrentBufferSize(),1,f);
		fclose(f);
	}
}

void TestJointTorqueSetup::stepSimulation(float deltaTime)
{
	m_multiBody->addLinkForce(0,btVector3(100,100,100));
    if (0)//m_once)
    {
       m_once=false;
        m_multiBody->addJointTorque(0, 10.0);
    
        btScalar torque = m_multiBody->getJointTorque(0);
        b3Printf("t = %f,%f,%f\n",torque,torque,torque);//[0],torque[1],torque[2]);
    }
    
    m_dynamicsWorld->stepSimulation(1./240,0);

	static int count = 0;
	if ((count& 0x0f)==0)
	{

	for (int i=0;i<m_jointFeedbacks.size();i++)
	{
			b3Printf("F_reaction[%i] linear:%f,%f,%f, angular:%f,%f,%f",
			i,
			m_jointFeedbacks[i]->m_reactionForces.m_topVec[0],
			m_jointFeedbacks[i]->m_reactionForces.m_topVec[1],
			m_jointFeedbacks[i]->m_reactionForces.m_topVec[2],

		m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[0],
			m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[1],
			m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[2]

		);

	}
	}
	count++;


	/*
    b3Printf("base angvel = %f,%f,%f",m_multiBody->getBaseOmega()[0],
             m_multiBody->getBaseOmega()[1],
             m_multiBody->getBaseOmega()[2]
             );
    */
   // btScalar jointVel =m_multiBody->getJointVel(0);
    
//    b3Printf("child angvel = %f",jointVel);
    
    
    
}


class CommonExampleInterface*    TestJointTorqueCreateFunc(struct CommonExampleOptions& options)
{
	return new TestJointTorqueSetup(options.m_guiHelper);
}
