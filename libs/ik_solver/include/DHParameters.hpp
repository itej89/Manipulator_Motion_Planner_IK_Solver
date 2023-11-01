#pragma once

/**
 * @file DHParameters.cpp
 *
 * @brief Represents Denavit-Hartenberg parameters for a robotic link.
 *
 * The Denavit-Hartenberg parameters are used to describe the geometry and joint
 * configurations of robotic manipulators. This class provides a convenient way
 * to store and access these parameters.
 *
 * @author Krishna Rajesh Hundekari
* Apache License Version 2.0, January 2004

 * Licensed to the Apache Software Foundation (ASF) under one
* or more contributor license agreements.  See the NOTICE file
* distributed with this work for additional information
* regarding copyright ownership.  The ASF licenses this file
* to you under the Apache License, Version 2.0 (the
* "License"); you may not use this file except in compliance
* with the License.  You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
* KIND, either express or implied.  See the License for the
* specific language governing permissions and limitations
* under the License.
 */

#include <iostream>
#include <vector>

class DHParameters {

 private:
  double d;
  double alpha;
  double a;
  double theta;

 public:

  friend class Robot;

  /**
   * @brief Parameterized constructor to initialize the DH parameters.
   * @param d The length of the link.
   * @param alpha The twist angle of the link.
   * @param a The offset along the link's z-axis.
   */
  DHParameters(double length, double twist, double offset)
      : d(length),
        alpha(twist),
        a(offset) {}

  /**
   * @brief Default Constructor for DHParameters
   * 
   */
  DHParameters() {}

  /**
   * @brief Get the length of the link.
   * @return The length of the link.
   */
  double getLinkLength() const;

  /**
   * @brief Get the twist angle of the link.
   * @return The twist angle of the link.
   */
  double getLinkTwist() const;

  /**
   * @brief Get the offset along the link's z-axis.
   * @return The offset along the link's z-axis.
   */
  double getLinkOffset() const;

};
