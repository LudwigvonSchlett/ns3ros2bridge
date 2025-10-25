/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHCOMM_H
#define ROSVEHCOMM_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"
#include "ns3/address.h"
#include "ns3/ipv4-address.h"
#include "ns3/inet-socket-address.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/pointer.h"
#include "ns3/object-vector.h"
#include "ns3/object-ptr-container.h"
#include <ns3/double.h>

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/wave-module.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/csma-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"  // for bulk send 
#include "ns3/tap-fd-net-device-helper.h" // for bulk send
#include "ns3/object-map.h"
#include <functional>
#include <stdlib.h>

#include <unordered_map>

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/netanim-module.h"

namespace ns3 {

class Socket;
class Packet;


class ROSVehSync : public Application 
{

public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  ROSVehSync ();
  virtual ~ROSVehSync ();


  uint64_t GetTotalRx () const;

  /**
   * \return pointer to listening socket
   */
  Ptr<Socket> GetListeningSocket (void) const;

  /**
   * \return list of pointers to accepted sockets
   */
  std::list<Ptr<Socket> > GetAcceptedSockets (void) const;

  typedef void (* SeqTsSizeCallback)(Ptr<const Packet> p, const Address &from, const Address & to,
                                   const SeqTsSizeHeader &header);


protected:
  virtual void DoDispose (void);

private:
	// Attributes

	Time m_interval; //Time to wait between packets

	Address ros_ip; // ROS Address
    uint16_t port; // socket port
	Address tap_ip;	// Tap device IP

    Ptr<Socket> controlSocket; // Socket with ROS for controller node
    TypeId controlSocket_tid; // socket protocol TypeId

	std::vector<Ptr<ConstantVelocityMobilityModel>> vector_mobilities;

    Ptr<YansWifiChannel> sharedChannel;


  virtual void StartApplication (void);
  virtual void StopApplication (void);

  /**
  * \brief Schedule the next packet transmission
  * \param dt time interval between packets.
  */
  void ScheduleArtemipsTransmit (Time dt);
  
  /**
  * \brief Send a packet
  */
  void SendArtemips (void);
  void HandleRead1 (Ptr<Socket> socket);

  void CreateVehicle (int i, double x, double y, double z, double xs, double ys, double zs);


  
  // Ptr<ConstantVelocityMobilityModel> m_m1; // For mobility of node1
  // Ptr<ConstantVelocityMobilityModel> m_m2; // For mobility of node1


  Ptr<Socket> m_socket_from_rtmaps; //!< IPv4 Socket

  EventId m_sendEvent_rtmaps; //!< Event to send the next packet



  /// Callbacks for tracing the packet Tx events
  TracedCallback<Ptr<const Packet> > m_txTrace;

  /* BELOW : COPIED FROM SINK */

  std::vector<std::string> SplitCharPointerController(const char* input);

  void HandleRead (Ptr<Socket> socket);

  void HandleAccept (Ptr<Socket> socket, const Address& from);
  /**
   * \brief Handle an connection close
   * \param socket the connected socket
   */
  void HandlePeerClose (Ptr<Socket> socket);
  /**
   * \brief Handle an connection error
   * \param socket the connected socket
   */
  void HandlePeerError (Ptr<Socket> socket);

  void PacketReceived (const Ptr<Packet> &p, const Address &from, const Address &localAddress);

    struct AddressHash
  {
    /**
     * \brief operator ()
     * \param x the address of which calculate the hash
     * \return the hash of x
     *
     * Should this method go in address.h?
     *
     * It calculates the hash taking the uint32_t hash value of the ipv4 address.
     * It works only for InetSocketAddresses (Ipv4 version)
     */
    size_t operator() (const Address &x) const
    {
      NS_ABORT_IF (!InetSocketAddress::IsMatchingType (x));
      InetSocketAddress a = InetSocketAddress::ConvertFrom (x);
      return std::hash<uint32_t>()(a.GetIpv4 ().Get ());
    }
  };

  std::unordered_map<Address, Ptr<Packet>, AddressHash> m_buffer; //!< Buffer for received packets



  std::list<Ptr<Socket> > m_socketList; //!< the accepted sockets


  uint64_t        m_totalRx;      //!< Total bytes received
  

  bool            m_enableSeqTsSizeHeader {false}; //!< Enable or disable the export of SeqTsSize header 

  /// Traced Callback: received packets, source address.
  TracedCallback<Ptr<const Packet>, const Address &> m_rxTrace;
  /// Callback for tracing the packet Rx events, includes source and destination addresses
  TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses;
  /// Callbacks for tracing the packet Rx events, includes source, destination addresses, and headers
  TracedCallback<Ptr<const Packet>, const Address &, const Address &, const SeqTsSizeHeader&> m_rxTraceWithSeqTsSize;

};

class ROSVehicule : public Application
{
public:
  static TypeId GetTypeId1 (void);
  ROSVehicule ();
  virtual ~ROSVehicule ();
  uint64_t GetTotalRx1 () const;
  Ptr<Socket> GetListeningSocket1 (void) const;
  std::list<Ptr<Socket> > GetAcceptedSockets1 (void) const;

typedef void (* SeqTsSizeCallback)(Ptr<const Packet> p, const Address &from, const Address & to,
                                    const SeqTsSizeHeader &header);
protected:
  virtual void DoDispose1 (void);
//-------------------------------------------Start of new changement-----------------------------

private:
   // inherited from Application base class.
   virtual void StartApplication (void);    // Called at time specified by Start
   virtual void StopApplication (void);     // Called at time specified by Stop

   std::vector<std::string> SplitCharPointer(const char* input);
   void HandleReadTapi (Ptr<Socket> socket);
   void HandleReadWavei (Ptr<Socket> socket);
   void HandleAccepti (Ptr<Socket> socket, const Address& from);
   void HandlePeerClosei (Ptr<Socket> socket);
   void HandlePeerErrori (Ptr<Socket> socket);
   void ScheduleArtemipsTransmiti (Time dt);
  
  void Replace_destination(Ptr<Socket> socket, Ptr<Packet> packet);

  void SendWithWave (Address waveSrcIP, Address waveDstIP, Ptr<Socket>socket, Ptr<Packet>packet);

  void ReceiveWave (Ptr<Socket> socket);

  void SendArtemips1 (void);

  void Send1(Ptr<Packet> packet);

  void PacketReceived1 (const Ptr<Packet> &p, const Address &from, const Address &localAddress);
 

   //void SendWave (const Address& from);

  EventId m_sendEvent_rtmaps1; //!< Event to send the next packet
  int m_nombre_vehicule1;

  Ptr< Packet> choice_dest; // for making choice to send data 

  Ptr<Socket> socket_send_RT;

   struct AddressHash1
   {
     size_t operator() (const Address &x) const
     {
       NS_ABORT_IF (!InetSocketAddress::IsMatchingType (x));
       InetSocketAddress a = InetSocketAddress::ConvertFrom (x);
       return std::hash<uint32_t>()(a.GetIpv4 ().Get ());
     }
   };
 
   std::unordered_map<Address, Ptr<Packet>, AddressHash1> m_buffer1; 
 
   // In the case of TCP, each socket accept returns a new socket, so the
   // listening socket is stored separately from the accepted sockets

   // ATTRIBUTES

   // Socket with tap device
   Ptr<Socket>     tapSocketi;
   TypeId          m_tapSocket_tidi;
   Address         tap_ipi;
   uint16_t portTapi; // socket port
   Address ros_ipi;

   Ptr<Socket>     waveSocketi;
   TypeId          m_waveSocket_tidi;
   uint16_t portWavei; // socket port
   Address         wave_ipi;




   std::list<Ptr<Socket> > m_socketList1;
   Address        m_adressewave;
   int         		vehicle_number;
   uint64_t        m_totalRx1;

   Address         copy_pkt;
   bool            m_enableSeqTsSizeHeader1 {false}; 
   Ptr<Socket>    m_socket_wave;
 
   TracedCallback<Ptr<const Packet>, const Address &> m_rxTrace1;
   TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses1;
   TracedCallback<Ptr<const Packet>, const Address &, const Address &, const SeqTsSizeHeader&> m_rxTraceWithSeqTsSize1;
  };

}
#endif /* ROSVEHCOMM_H */