/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHICULE_HELPER_H
#define ROSVEHICULE_HELPER_H

#include <stdint.h>
#include "ns3/rosvehicule.h"
#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

  class ROSVehiculeHelper
  {

    public:
      ROSVehiculeHelper ();

      void SetAttribute (std::string name, const AttributeValue &value);
      //void SetAttribute (std::string name, AddressValue adress);

      ApplicationContainer Install (Ptr<Node> node) const;

      ApplicationContainer Install (std::string nodeName) const;

      ApplicationContainer Install (NodeContainer c) const;

    private:

      Ptr<Application> InstallPriv (Ptr<Node> node) const;

      ObjectFactory m_factory;

  };
}

#endif /* ROSVEHICULE_HELPER_H */

