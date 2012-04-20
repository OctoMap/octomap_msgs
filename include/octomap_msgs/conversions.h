// $Id$

/**
 * OctoMap ROS integration
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

namespace octomap {
  /**
   * @brief Converts an octomap map structure to a ROS octomap msg as binary data.
   * This will fill the timestamp of the header with the current time, but will
   * not fill in the frame_id.
   *
   * @param octomap input OcTree
   * @param mapMsg output msg
   */
  template <class OctomapT>
  static inline void octomapMapToMsg(const OctomapT& octomap, octomap_msgs::OctomapBinary& mapMsg){
    mapMsg.header.stamp = ros::Time::now();

    octomapMapToMsgData(octomap, mapMsg.data);
  }

   /**
    * @brief Converts an octomap map structure to a ROS binary data, which can be
    * put into a dedicated octomap msg.
    *
    * @param octomap input OcTree
    * @param mapData binary output data as int8[]
    */
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
   * @brief Converts a ROS octomap msg (binary data) to an octomap map structure
   *
   * @param mapMsg
   * @param octomap
   */
  template <class OctomapT>
  static inline void octomapMsgToMap(const octomap_msgs::OctomapBinary& mapMsg, OctomapT& octomap){
	  octomapMsgDataToMap(mapMsg.data, octomap);
  }

  /**
   * @brief Converts ROS binary data to an octomap map structure, e.g. coming from an
   * octomap msg
   *
   * @param mapData input binary data
   * @param octomap output OcTree
   */
  template <class OctomapT>
  static inline void octomapMsgDataToMap(const std::vector<int8_t>& mapData, OctomapT& octomap){
    std::stringstream datastream;
    assert(mapData.size() > 0);
    datastream.write((const char*) &mapData[0], mapData.size());
    octomap.readBinary(datastream);
  }

}


#endif

