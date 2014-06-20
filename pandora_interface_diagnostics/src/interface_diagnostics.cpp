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


InterfaceDiagnostics::InterfaceDiagnostics() :
    GenericDiagnostic("System Interfaces"),
    StateClient(false) {

  docsVector_ = parser_.getDocsVector();

};

void InterfaceDiagnostics::startTransition (int newState){

  currentState_ = newState;
  transitionComplete(newState);
      }

void InterfaceDiagnostics::nodeDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){

  bool allOk = true;

  TiXmlElement * packageElement;
  TiXmlElement * nodeElement;
  TiXmlElement * tfElement;

  for(int ii=0;ii<docsVector_.size();ii++){

    //~ Suppose there is only one package element in each document
    packageElement = docsVector_[ii]->FirstChildElement("package");
    nodeElement = packageElement->FirstChildElement("node");

    while(nodeElement) {

      nodePublisherDiagnostic(nodeElement, stat, allOk);
      nodeSubscriberDiagnostic(nodeElement, stat, allOk);

      nodeActionServerDiagnostic(nodeElement, stat, allOk);
      nodeActionClientDiagnostic(nodeElement, stat, allOk);

      tfPublisherDiagnostic(nodeElement, stat, allOk);

      nodeElement = nodeElement->NextSiblingElement( "node" );
    }

    tfElement = packageElement->FirstChildElement("tf");
    while(tfElement) {
      tfTransformDiagnostic(tfElement, stat, allOk);
      tfElement = tfElement->NextSiblingElement( "tf" );
    }

    if (allOk) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
        "All interfaces are OK");
    } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
        "Some interfaces are OFF");
    }

  }
}

InterfaceDiagnostics::~InterfaceDiagnostics() {}

void InterfaceDiagnostics::nodePublisherDiagnostic(TiXmlElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat,bool& allOk){


  std::vector<std::string> children = getChildren(nodeElement,"publisher");
  std::string nodeName = nodeElement->Attribute("name");


  for (int ii=0; ii<children.size(); ++ii) {
    if (!InterfaceTester::checkForNodePublishing(children[ii],nodeName)){
      stat.add(nodeName + " is not publishing", children[ii]);
      allOk = false;
    }
  }

}

void InterfaceDiagnostics::nodeSubscriberDiagnostic(TiXmlElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk){

  std::vector<std::string> children = getChildren(nodeElement,"subscriber");
  std::vector<int> include_states;
  std::vector<int> exclude_states;

  if(nodeElement->Attribute("name")){

    std::string nodeName = nodeElement->Attribute("name");

    for (int ii = 0; ii<children.size(); ++ii){

      include_states = getStates(nodeElement,"subscriber","include_states");
      exclude_states = getStates(nodeElement,"subscriber","exclude_states");

      bool isIncluded = std::find(include_states.begin(),
        include_states.end(), currentState_)!=include_states.end();

      bool isExcluded = std::find(exclude_states.begin(),
        exclude_states.end(), currentState_)!=exclude_states.end();

      bool isUp = InterfaceTester::checkForSubscribedNode(children[ii],nodeName);

      if (isIncluded&&isExcluded){
           ROS_WARN("OMG THIS CANNOT HAPPEN");
      }
      else if(isIncluded && !isUp){
              stat.add(nodeName + " is not subscribed at", children[ii]);
              allOk = false;
      }
      else if (isIncluded && isUp){
             stat.add(nodeName + "should not be subscribed at", children[ii]);
             allOk = false;
      }
    }
  }
  else{
    ROS_WARN("Some Nodes dont have a name attribute");
  }

 }

void InterfaceDiagnostics::nodeActionServerDiagnostic(TiXmlElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk){
  std::vector<std::string> children =
  getChildren(nodeElement,"actionServer","serviceName");


  std::string nodeName = nodeElement->Attribute("name");


  for (int ii=0; ii<children.size(); ++ii) {
    if (!InterfaceTester::checkForActionNodeServer(children[ii],nodeName)){
      stat.add(nodeName + " is not providing action server", children[ii]);
      allOk = false;
    }
  }

}

void InterfaceDiagnostics::nodeActionClientDiagnostic(TiXmlElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk){

  std::vector<std::string> children = getChildren(nodeElement,"actionClient" , "actionName");
  std::string nodeName = nodeElement->Attribute("name");

  for (int ii=0; ii<children.size(); ++ii) {
    if (!InterfaceTester::checkForActionNodeClient(children[ii],nodeName)){
      stat.add(nodeName + " is not an action client to", children[ii]);
      allOk = false;
    }
  }

}

void InterfaceDiagnostics::tfPublisherDiagnostic(TiXmlElement* nodeElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk){

  std::vector<std::string> children = getChildren(nodeElement,"tf-publisher");
  std::string nodeName = nodeElement->Attribute("name");

  for (int ii=0; ii < children.size(); ++ ii) {
    if (!tfMonitor_.checkForPublishedTF(children[ii], nodeName)){
      stat.add(nodeName + " not publishing tf to frame", children[ii]);
      allOk = false;
    }
  }

}

void InterfaceDiagnostics::tfTransformDiagnostic(TiXmlElement* tfParentElement,
  diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk){
  std::vector<std::string> children = getChildren(tfParentElement,"tf-publisher");
  std::string tfParentName = tfParentElement->Attribute("name");

  for (int ii=0; ii<children.size(); ++ii) {
    if (!InterfaceTester::checkForTF(children[ii],tfParentName)){
      stat.add("Tf not published: ",
        tfParentName + " --> "+children[ii]);
      allOk = false;
    }
  }
}

std::vector<std::string> InterfaceDiagnostics::getChildren(
  TiXmlElement* parentElement,std::string type ,std::string attribute ){

  std::vector<std::string> children;

  TiXmlElement* currentElement = parentElement->FirstChildElement(type);
  while(currentElement){

    if (currentElement->Attribute("optional")){
      ROS_ERROR("%s",currentElement->Attribute("optional"));
      if (!(currentElement->Attribute("optional") == "true")){
        children.push_back(currentElement->Attribute(attribute.c_str()));
      }
    }
    else {
      ROS_WARN("Some children dont have an optional attribute");
    }

    currentElement = currentElement->NextSiblingElement(type);
  }
  return children;
}

std::vector<int> InterfaceDiagnostics::getStates(
  TiXmlElement* parentElement,std::string type ,std::string type2  ){

  std::vector<int>  states;

  TiXmlElement* currentElement = parentElement->FirstChildElement(type);
   
  std::string maria = currentElement->Attribute(type2.c_str());
  states = (stringToIntiger(strdup(maria.c_str())));
  
  return states;
}


std::vector<int> InterfaceDiagnostics::stringToIntiger( char* strList){
    string eValue;
    std::vector<int> list;

    for(int ii = 0; ii < strlen(strList); ii++){

       if(strList[ii]!=','){
           eValue.push_back(strList[ii]);
       }
       else{
          int element = atoi(eValue.c_str());
          ROS_ERROR("%d",element);
          eValue.clear();
          list.push_back(element);
       }
    }
    return list;
}




