/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHICULE_H
#define ROSVEHICULE_H

#include <functional>
#include <iomanip>
#include <unordered_map>

#include "ns3/core-module.h"
#include "ns3/event-id.h"
#include "ns3/network-module.h"
#include "ns3/ptr.h"
#include "ns3/rosvehsync-helper.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/traced-callback.h"
#include "ns3/wave-module.h"
#include "rosheader.h"

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

      void HandleReadTap (Ptr<Socket> socket);
      void HandleReadWave (Ptr<Socket> socket);

      // ATTRIBUTES

      // Socket with tap device
      Ptr<Socket>     tapSocketi;
      TypeId          m_tapSocket_tidi;
      Address         tap_ipi;
      uint16_t        portTapi; // socket port
      Address         ros_ipi;

      Ptr<Socket>     waveSocketi;
      TypeId          m_waveSocket_tidi;
      uint16_t        portWavei; // socket port
      Address         wave_ipi;

      int         		  vehicle_number;
      uint64_t          m_totalRx1;

      bool            m_enableSeqTsSizeHeader1 {false};

      TracedCallback<Ptr<const Packet>, const Address &> m_rxTrace1;
      TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses1;
      TracedCallback<Ptr<const Packet>, const Address &, const Address &, const SeqTsSizeHeader&> m_rxTraceWithSeqTsSize1;
  };

}
#endif /* ROSVEHCOMM_H */