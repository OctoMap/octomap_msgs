// $Id$

/**
 * OctoMap ROS message conversions / [de-] serialization
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011-2012.
 * @see http://www.ros.org/wiki/octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_MSGS_CONVERT_MSGS_H
#define OCTOMAP_MSGS_CONVERT_MSGS_H

#include <octomap/octomap.h>
#include <octomap_msgs/OctomapBinary.h>

// new conversion functions  
namespace octomap_msgs{
  static inline octomap::AbstractOcTree* fullMsgDataToMap(const std::vector<int8_t>& mapData){
    std::stringstream datastream;
    assert(mapData.size() > 0);
    datastream.write((const char*) &mapData[0], mapData.size());
    return octomap::AbstractOcTree::read(datastream);
  }
  
  static inline octomap::OcTree* binaryMsgDataToMap(const std::vector<int8_t>& mapData){
    octomap::OcTree* octree = new octomap::OcTree(0.1);
    std::stringstream datastream;
    assert(mapData.size() > 0);
    datastream.write((const char*) &mapData[0], mapData.size());
    octree->readBinary(datastream);
    return octree;
  }
  
  // conversions via stringstream
  
  // TODO: read directly into buffer? see
  // http://stackoverflow.com/questions/132358/how-to-read-file-content-into-istringstream
  
  template <class OctomapT>
  static inline bool binaryMapToMsgData(const OctomapT& octomap, std::vector<int8_t>& mapData){
    std::stringstream datastream;
    if (!octomap.writeBinaryConst(datastream))
      return false;
    
    std::string datastring = datastream.str();
    mapData = std::vector<int8_t>(datastring.begin(), datastring.end());
    return true;
  }
  
  template <class OctomapT>
  static inline bool fullMapToMsgData(const OctomapT& octomap, std::vector<int8_t>& mapData){
    std::stringstream datastream;
    if (!octomap.write(datastream))
      return false;
    
    std::string datastring = datastream.str();
    mapData = std::vector<int8_t>(datastring.begin(), datastring.end());
    return true;
  }

}

// deprecated old conversion functions, use the ones above instead!
namespace octomap {
  /**
   * @brief Deprecated, use octomap_msgs::binaryMapToMsgData() instead
   */
  template <class OctomapT>
  static inline void octomapMapToMsg(const OctomapT& octomap, octomap_msgs::OctomapBinary& mapMsg) __attribute__ ((deprecated));
  
  template <class OctomapT>
  static inline void octomapMapToMsg(const OctomapT& octomap, octomap_msgs::OctomapBinary& mapMsg){
    mapMsg.header.stamp = ros::Time::now();
    
    octomapMapToMsgData(octomap, mapMsg.data);
  }
  
  /**
   * @brief Deprecated, use octomap_msgs::binaryMapToMsgData() instead
   */
  template <class OctomapT>
  static inline void octomapMapToMsgData(const OctomapT& octomap, std::vector<int8_t>& mapData) __attribute__ ((deprecated));
  
  template <class OctomapT>
  static inline void octomapMapToMsgData(const OctomapT& octomap, std::vector<int8_t>& mapData){
    // conversion via stringstream
    
    // TODO: read directly into buffer? see
    // http://stackoverflow.com/questions/132358/how-to-read-file-content-into-istringstream
    std::stringstream datastream;
    octomap.writeBinaryConst(datastream);
    std::string datastring = datastream.str();
    mapData = std::vector<int8_t>(datastring.begin(), datastring.end());
  }
  
  
  /**
   * @brief Deprecated, use octomap_msgs::binaryMsgDataToMap() instead
   */
  template <class OctomapT>
  static inline void octomapMsgToMap(const octomap_msgs::OctomapBinary& mapMsg, OctomapT& octomap) __attribute__ ((deprecated));
  
  template <class OctomapT>
  static inline void octomapMsgToMap(const octomap_msgs::OctomapBinary& mapMsg, OctomapT& octomap) {
    octomapMsgDataToMap(mapMsg.data, octomap);
  }
  
  /**
   * @brief Deprecated, use octomap_msgs::binaryMsgDataToMap() instead
   */
  template <class OctomapT>
  static inline void octomapMsgDataToMap(const std::vector<int8_t>& mapData, OctomapT& octomap) __attribute__ ((deprecated));
  
  
  template <class OctomapT>
  static inline void octomapMsgDataToMap(const std::vector<int8_t>& mapData, OctomapT& octomap){
    std::stringstream datastream;
    assert(mapData.size() > 0);
    datastream.write((const char*) &mapData[0], mapData.size());
    octomap.readBinary(datastream);
  }
}


#endif

