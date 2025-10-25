/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "rosvehcomm-helper.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

ROSVehSyncHelper::ROSVehSyncHelper ()
{
  m_factory.SetTypeId (ROSVehSync::GetTypeId ());
}

void 
ROSVehSyncHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
ROSVehSyncHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
ROSVehSyncHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
ROSVehSyncHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
ROSVehSyncHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<ROSVehSync> ();
  node->AddApplication (app);

  return app;
}


//------------------------------------------


ROSVehicule1Helper::ROSVehicule1Helper ()
{
  m_factory.SetTypeId (ROSVehicule::GetTypeId1 ());
}

void 
ROSVehicule1Helper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
  //m_factory.Set ("Protocol",StringValue(protocol));
  //m_factory.Set ("Local", AddressValue (address));
}


ApplicationContainer
ROSVehicule1Helper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
ROSVehicule1Helper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
ROSVehicule1Helper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
ROSVehicule1Helper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<ROSVehicule> ();
  node->AddApplication (app);

  return app;
}


}

