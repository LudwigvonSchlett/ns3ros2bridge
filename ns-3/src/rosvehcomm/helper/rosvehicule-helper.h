/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHICULE_HELPER_H
#define ROSVEHICULE_HELPER_H

#include "ns3/rosvehicule.h"

namespace ns3 {

  class ROSVehiculeHelper
  {

    public:
      ROSVehiculeHelper ();

      void SetAttribute (std::string name, const AttributeValue &value);

      ApplicationContainer Install (Ptr<Node> node) const;

      ApplicationContainer Install (std::string nodeName) const;

      ApplicationContainer Install (NodeContainer c) const;

    private:

      Ptr<Application> InstallPriv (Ptr<Node> node) const;

      ObjectFactory m_factory;

  };
}

#endif /* ROSVEHICULE_HELPER_H */

