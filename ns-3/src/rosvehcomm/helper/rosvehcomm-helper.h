/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSVEHCOMM_HELPER_H
#define ROSVEHCOMM_HELPER_H

#include "ns3/rosvehsync.h"
#include "ns3/rosvehicule.h"
#include <stdint.h>
#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"





namespace ns3 {

  class ROSVehSyncHelper
  {
    public:

      ROSVehSyncHelper ();
      /**
       * Record an attribute to be set in each Application after it is is created.
       *
       * \param name the name of the attribute to set
       * \param value the value of the attribute to set
       */
      void SetAttribute (std::string name, const AttributeValue &value);

      /**
       * Create a TrafficInfoServerApplication on the specified Node.
       *
       * \param node The node on which to create the Application.  The node is
       *             specified by a Ptr<Node>.
       *
       * \returns An ApplicationContainer holding the Application created,
       */
      ApplicationContainer Install (Ptr<Node> node) const;

      /**
       * Create a TrafficInfoServerApplication on specified node
       *
       * \param nodeName The node on which to create the application.  The node
       *                 is specified by a node name previously registered with
       *                 the Object Name Service.
       *
       * \returns An ApplicationContainer holding the Application created.
       */
      ApplicationContainer Install (std::string nodeName) const;

      /**
       * \param c The nodes on which to create the Applications.  The nodes
       *          are specified by a NodeContainer.
       *
       * Create one Traffic Info server application on each of the Nodes in the
       * NodeContainer.
       *
       * \returns The applications created, one Application per Node in the
       *          NodeContainer.
       */
      ApplicationContainer Install (NodeContainer c) const;

    private:
      /**
       * Install an ns3::TrafficInfoServer on the node configured with all the
       * attributes set with SetAttribute.
       *
       * \param node The node on which an TrafficInfoServer will be installed.
       * \returns Ptr to the application installed.
       */
      Ptr<Application> InstallPriv (Ptr<Node> node) const;

      ObjectFactory m_factory; //!< Object factory.
  };

  class ROSVehicule1Helper
  {

    public:
      ROSVehicule1Helper ();

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

#endif /* ROSVEHCOMM_HELPER_H */

