// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.h
 * @brief Service implementation header of MobileRobot.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSkel.h"

#include "Mapper_MRPT.h"

#ifndef MOBILEROBOTSVC_IMPL_H
#define MOBILEROBOTSVC_IMPL_H
 
/*!
 * @class OGMapperSVC_impl
 * Example class implementing IDL interface RTC::OGMapper
 */
class OGMapperSVC_impl
 : public virtual POA_RTC::OGMapper,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~OGMapperSVC_impl();

	 Mapper_MRPT *m_pRTC;

 public:

	 void setMapperRTC(Mapper_MRPT* pRTC) {m_pRTC = pRTC;}

  /*!
   * @brief standard constructor
   */
   OGMapperSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~OGMapperSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE initializeMap(const RTC::OGMapConfig& config, const RTC::Pose2D& initialPose);
   RTC::RETURN_VALUE startMapping();
   RTC::RETURN_VALUE stopMapping();
   RTC::RETURN_VALUE suspendMapping();
   RTC::RETURN_VALUE resumeMapping();
  RTC::RETURN_VALUE getState(RTC::MAPPER_STATE& state);
  RTC::RETURN_VALUE requestCurrentBuiltMap(RTC::OGMap_out map);

};



#endif // MOBILEROBOTSVC_IMPL_H


