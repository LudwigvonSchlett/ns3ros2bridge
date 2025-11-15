/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHICULE_H
#define ROSVEHICULE_H

#include "ns3/rosvehsync-helper.h"
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

  class ROSVehicule : public Application
  {
    public:
      static TypeId GetTypeId (void);
      ROSVehicule ();
      virtual ~ROSVehicule ();

    typedef void (* SeqTsSizeCallback)(Ptr<const Packet> p, const Address &from, const Address & to,
                                        const SeqTsSizeHeader &header);
    protected:
      virtual void DoDispose (void);
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

      EventId m_sendEvent_rtmaps1; //!< Event to send the next packet
      int m_nombre_vehicule1;

      Ptr<Packet> choice_dest; // for making choice to send data

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