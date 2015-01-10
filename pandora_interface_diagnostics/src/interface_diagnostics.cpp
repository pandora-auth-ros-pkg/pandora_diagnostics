/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
*   Nikos Gountas
*   Triantafyllos Afouras <afourast@gmail.com>
*********************************************************************/

#include "pandora_interface_diagnostics/interface_diagnostics.h"

InterfaceDiagnostics::InterfaceDiagnostics() : GenericDiagnostic("System Interfaces"), StateClient(false) 
{
  currentState_ = 0;
  ParentElement_ = new InfoElement();
  packages_= ParentElement_->getAllElements();
}

InterfaceDiagnostics::~InterfaceDiagnostics() 
{
}

void InterfaceDiagnostics::startTransition(int newState)
{
  currentState_ = newState;
  transitionComplete(newState);
}

void InterfaceDiagnostics::nodeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  bool allOk = true;
  
  std::vector<InfoElement*> nodeElements;
  std::vector<InfoElement*> tfElements;
  
  for (int ii = 0; ii < packages_.size(); ii++)
  {
    nodeElements = packages_[ii]->getChildren("node");
    for (int jj = 0; jj < nodeElements.size(); jj++)
    {
      nodePublisherDiagnostic(nodeElements[jj], stat, allOk);
      nodeSubscriberDiagnostic(nodeElements[jj], stat, allOk);

      nodeActionServerDiagnostic(nodeElements[jj], stat, allOk);
      nodeActionClientDiagnostic(nodeElements[jj], stat, allOk);

      // tfPublisherDiagnostic(nodeElements[jj], stat, allOk);
    }
    
    tfElements = packages_[ii]->getChildren("tf");
    for (int jj = 0; jj < tfElements.size(); jj++)
    {
      tfTransformDiagnostic(tfElements[jj], stat, allOk);
    }
    
    if (allOk) 
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All interfaces are OK");
    } 
    else 
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Some interfaces are OFF");
    }
  }
}

void InterfaceDiagnostics::nodePublisherDiagnostic(InfoElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat,bool& allOk)
{
  std::vector<std::string> publishers = getChildren(nodeElement, "publisher");
  std::vector<int> include_states;
  std::vector<int> exclude_states;
  
  if (!nodeElement->Attribute("name").empty())
  {
    std::string nodeName = nodeElement->Attribute("name");
    
    for (int ii = 0; ii < publishers.size(); ++ii) 
    {
      include_states = getStates(nodeElement, "publisher", "include_states");
      exclude_states = getStates(nodeElement, "publisher", "exclude_states");

      bool isIncluded = std::find(include_states.begin(), include_states.end(), currentState_) != include_states.end();
      bool isExcluded = std::find(exclude_states.begin(), exclude_states.end(), currentState_) != exclude_states.end();
      bool isUp = InterfaceTester::checkForNodePublishing(publishers[ii], nodeName);

      if (isIncluded && isExcluded)
      {
        ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if (isIncluded && !isUp)
      {
        stat.add(nodeName + " is not publishing", publishers[ii]);
        allOk = false;
      }
      else if (!isIncluded && isUp)
      {
        stat.add(nodeName + "should not publish", publishers[ii]);
        allOk = false;
      }
    }
  }
  else
  {
    ROS_WARN("Some Nodes dont have a name attribute");
  }
}


void InterfaceDiagnostics::nodeSubscriberDiagnostic(InfoElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk)
{
  std::vector<std::string> subscribers = getChildren(nodeElement, "subscriber");
  std::vector<int> include_states;
  std::vector<int> exclude_states;

  if (!nodeElement->Attribute("name").empty())
  {
    std::string nodeName = nodeElement->Attribute("name");

    for (int ii = 0; ii < subscribers.size(); ++ii)
    {
      include_states = getStates(nodeElement, "subscriber", "include_states");
      exclude_states = getStates(nodeElement, "subscriber", "exclude_states");

      bool isIncluded = std::find(include_states.begin(), include_states.end(), currentState_) != include_states.end();
      bool isExcluded = std::find(exclude_states.begin(), exclude_states.end(), currentState_) != exclude_states.end();
      bool isUp = InterfaceTester::checkForSubscribedNode(subscribers[ii], nodeName);

      if (isIncluded && isExcluded)
      {
        ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if (isIncluded && !isUp)
      {
        stat.add(nodeName + " is not subscribed at", subscribers[ii]);
        allOk = false;
      }
      else if (!isIncluded && isUp)
      {
        stat.add(nodeName + "should not be subscribed at", subscribers[ii]);
        allOk = false;
      }
    }
  }
  else
  {
    ROS_WARN("Some Nodes dont have a name attribute");
  }
}

void InterfaceDiagnostics::nodeActionServerDiagnostic(InfoElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk)
{
  std::vector<std::string> actionServers = getChildren(nodeElement, "actionServer", "serviceName");
  std::vector<int> include_states;
  std::vector<int> exclude_states;
  
  if (!nodeElement->Attribute("name").empty())
  {
    std::string nodeName = nodeElement->Attribute("name");
    
    for (int ii = 0; ii < actionServers.size(); ++ii) 
    {
      include_states = getStates(nodeElement, "actionServer", "include_states");
      exclude_states = getStates(nodeElement, "actionServer", "exclude_states");

      bool isIncluded = std::find(include_states.begin(), include_states.end(), currentState_) != include_states.end();
      bool isExcluded = std::find(exclude_states.begin(), exclude_states.end(), currentState_) != exclude_states.end();
      bool isUp = InterfaceTester::checkForActionNodeServer(actionServers[ii], nodeName);

      if (isIncluded && isExcluded)
      {
        ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if (isIncluded && !isUp)
      {
        stat.add(nodeName + " is not providing action server", actionServers[ii]);
        allOk = false;
      }
      else if (!isIncluded && isUp)
      {
        stat.add(nodeName + "should not provide action server", actionServers[ii]);
        allOk = false;
      }
    }
  }
  else
  {
    ROS_WARN("Some Nodes dont have a name attribute");
  }
}

void InterfaceDiagnostics::nodeActionClientDiagnostic(InfoElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk)
{
  std::vector<std::string> actionClients = getChildren(nodeElement, "actionClient" , "actionName");
  std::vector<int> include_states;
  std::vector<int> exclude_states;
  
  if (!nodeElement->Attribute("name").empty())
  {
    std::string nodeName = nodeElement->Attribute("name");
    
    for (int ii = 0; ii < actionClients.size(); ++ii) 
    {
      include_states = getStates(nodeElement, "actionClient", "include_states");
      exclude_states = getStates(nodeElement, "actionClient", "exclude_states");

      bool isIncluded = std::find(include_states.begin(), include_states.end(), currentState_) != include_states.end();
      bool isExcluded = std::find(exclude_states.begin(), exclude_states.end(), currentState_) != exclude_states.end();
      bool isUp = InterfaceTester::checkForActionNodeClient(actionClients[ii], nodeName);

      if (isIncluded && isExcluded)
      {
        ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if (isIncluded && !isUp)
      {
        stat.add(nodeName + " is not an action client to", actionClients[ii]);
        allOk = false;
      }
      else if (!isIncluded && isUp)
      {
        stat.add(nodeName + "should not be an action client to", actionClients[ii]);
        allOk = false;
      }
    }
  }
  else
  {
    ROS_WARN("Some Nodes dont have a name attribute");
  }
}

/*
void InterfaceDiagnostics::tfPublisherDiagnostic(InfoElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk)
{
  std::vector<std::string> children = getChildren(nodeElement, "tf-publisher");
  std::vector<int> include_states;
  std::vector<int> exclude_states;
  
  if (!nodeElement->Attribute("name").empty())
  {
    std::string nodeName = nodeElement->Attribute("name");
    
    for (int ii = 0; ii < children.size(); ++ii) 
    {
      include_states = getStates(nodeElement, "tf-publisher", "include_states");
      exclude_states = getStates(nodeElement, "tf-publisher", "exclude_states");

      bool isIncluded = std::find(include_states.begin(), include_states.end(), currentState_) != include_states.end();
      bool isExcluded = std::find(exclude_states.begin(), exclude_states.end(), currentState_) != exclude_states.end();
      bool isUp = InterfaceTester::checkForPublishedTF(children[ii], nodeName);

      if (isIncluded && isExcluded)
      {
        ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if (isIncluded && !isUp)
      {
        stat.add(nodeName + " not publishing tf to frame", children[ii]);
        allOk = false;
      }
      else if (!isIncluded && isUp)
      {
        stat.add(nodeName + "should not publish tf to frame", children[ii]);
        allOk = false;
      }
    }
  }
  else
  {
    ROS_WARN("Some Nodes dont have a name attribute");
  }
}
*/

void InterfaceDiagnostics::tfTransformDiagnostic(InfoElement* tfParentElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk)
{
  std::vector<std::string> children = getChildren(tfParentElement, "tf-publisher");
  std::vector<int> include_states;
  std::vector<int> exclude_states;
  
  if (!tfParentElement->Attribute("name").empty())
  {
    std::string tfParentName = tfParentElement->Attribute("name");
    
    for (int ii = 0; ii < children.size(); ++ii) 
    {
      include_states = getStates(tfParentElement, "tf-publisher", "include_states");
      exclude_states = getStates(tfParentElement, "tf-publisher", "exclude_states");

      bool isIncluded = std::find(include_states.begin(), include_states.end(), currentState_) != include_states.end();
      bool isExcluded = std::find(exclude_states.begin(), exclude_states.end(), currentState_) != exclude_states.end();
      bool isUp = InterfaceTester::checkForTF(children[ii], tfParentName);

      if (isIncluded && isExcluded)
      {
        ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if (isIncluded && !isUp)
      {
        stat.add("Tf not published: ", tfParentName + " --> " + children[ii]);
        allOk = false;
      }
      else if (!isIncluded && isUp)
      {
        stat.add("Tf should not be published: ", tfParentName + " --> " + children[ii]);
        allOk = false;
      }
    }
  }
  else
  {
    ROS_WARN("Some Nodes dont have a name attribute");
  }
}


std::vector<std::string> InterfaceDiagnostics::getChildren(InfoElement* parentElement, std::string type,
  std::string attribute)
{
  std::vector<std::string> attributes;
  
  std::vector<InfoElement*> children = parentElement->getChildren(type);
  for (int ii = 0; ii < children.size(); ii++)
  {
    if (!children[ii]->Attribute("optional").empty())
    {
      if (string("true").compare(children[ii]->Attribute("optional")))
      {
        attributes.push_back(children[ii]->Attribute(attribute.c_str()));
      }
    }
    else 
    {
      ROS_WARN("Some children dont have an optional attribute");
    }
  }
  return attributes;
}

std::vector<int> InterfaceDiagnostics::getStates(InfoElement* parentElement, std::string type, std::string statesType)
{
  std::vector<int> states;
  std::vector<InfoElement*> children = parentElement->getChildren(type);
  states = stringToInteger(children[0]->Attribute(statesType.c_str()));
  return states;
}

std::vector<int> InterfaceDiagnostics::stringToInteger(std::string sample)
{
  std::vector<int> list;
  std::vector<std::string> strs;
  boost::split(strs, sample, boost::is_any_of(","));

  for (int ii = 0; ii < strs.size(); ii++)
  {
    int element = atoi(strs[ii].c_str());
    ROS_ERROR("%d", element);
    ROS_ERROR("%s", strs[ii].c_str());
    list.push_back(element);
  }
  return list;
}

