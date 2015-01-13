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
*   Chamzas Konstantinos <chamzask@gmail.com>
*********************************************************************/

#include <string>
#include <vector>
#include "pandora_interface_diagnostics/info_element.h"

InfoElement::InfoElement(){
  docsVector_ = parser_.getDocsVector();
  XmlElement_ = NULL;
}

InfoElement::InfoElement(TiXmlElement* newElement){
  docsVector_.push_back(NULL);
  XmlElement_ = newElement;
}

InfoElement::~InfoElement(){
}

std::vector<InfoElement*> InfoElement::getAllElements(){
  std::vector<InfoElement*> elements;

  for (int ii = 0; ii < docsVector_.size(); ii++){
    InfoElement* newElement = new InfoElement(
      docsVector_[ii]->FirstChildElement("package"));
    elements.push_back(newElement);
  }
  return elements;
}

std::vector<InfoElement*> InfoElement::getChildren(std::string type){
  std::vector<InfoElement*> children;

  TiXmlElement* currentElement = XmlElement_->FirstChildElement(type);
  
  while (currentElement){
    if (currentElement->Attribute("optional")){
      if (string("true").compare(currentElement->Attribute("optional"))){
        InfoElement* newElement = new InfoElement(currentElement);
        children.push_back(newElement);
      }
    }
    else {
      ROS_WARN("Some children dont have an optional attribute");
    }
    currentElement = currentElement->NextSiblingElement(type);
  }
  return children;
}

std::string InfoElement::Attribute(std::string attribute){
  return XmlElement_->Attribute(attribute.c_str());
}

