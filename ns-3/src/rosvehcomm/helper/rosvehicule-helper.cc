/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "rosvehicule-helper.h"

namespace ns3 {

  ROSVehiculeHelper::ROSVehiculeHelper ()
  {
    m_factory.SetTypeId (ROSVehicule::GetTypeId1 ());
  }

  void
  ROSVehiculeHelper::SetAttribute (std::string name, const AttributeValue &value)
  {
    m_factory.Set (name, value);
    //m_factory.Set ("Protocol",StringValue(protocol));
    //m_factory.Set ("Local", AddressValue (address));
  }

  ApplicationContainer
  ROSVehiculeHelper::Install (Ptr<Node> node) const
  {
    return ApplicationContainer (InstallPriv (node));
  }

  ApplicationContainer
  ROSVehiculeHelper::Install (std::string nodeName) const
  {
    Ptr<Node> node = Names::Find<Node> (nodeName);
    return ApplicationContainer (InstallPriv (node));
  }

  ApplicationContainer
  ROSVehiculeHelper::Install (NodeContainer c) const
  {
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

    return apps;
  }

  Ptr<Application>
  ROSVehiculeHelper::InstallPriv (Ptr<Node> node) const
  {
    Ptr<Application> app = m_factory.Create<ROSVehicule> ();
    node->AddApplication (app);

    return app;
  }
}

