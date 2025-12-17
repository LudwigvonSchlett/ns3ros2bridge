/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHSYNC_H
#define ROSVEHSYNC_H

#include <functional>
#include <unordered_map>

#include "ns3/applications-module.h"  // for bulk send
#include "ns3/core-module.h"
#include "ns3/event-id.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ptr.h"
#include "ns3/rosvehicule-helper.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/tap-fd-net-device-helper.h" // for bulk send
#include "ns3/traced-callback.h"
#include "ns3/wave-module.h"

namespace ns3 {

  class Socket;
  class Packet;

  struct SimInfo {
      std::string filename;
      Time duration;
  };

  extern SimInfo simInfo;

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

      Ptr<Socket> m_socket_from_rtmaps; //!< IPv4 Socket

      EventId m_sendEvent_rtmaps; //!< Event to send the next packet

      /// Callbacks for tracing the packet Tx events
      TracedCallback<Ptr<const Packet> > m_txTrace;

      /* BELOW : COPIED FROM SINK */

      static std::vector<std::string> SplitCharPointerController(const char* input);

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
}

#endif /* ROSVEHSYNC_H */