/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#ifndef LATEST_POLICY_H_
#define LATEST_POLICY_H_

#include <cassert>
#include <deque>
#include <string>
#include <tuple>
#include <vector>

#include <inttypes.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include "message_filters/connection.h"
#include "message_filters/message_traits.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/synchronizer.h"


#ifndef RCUTILS_ASSERT
// TODO(tfoote) remove this after it's implemented upstream
// https://github.com/ros2/rcutils/pull/112
#define RCUTILS_ASSERT assert
#endif
#ifndef RCUTILS_BREAK
#include <cassert>
// TODO(tfoote) remove this after it's implemented upstream
// https://github.com/ros2/rcutils/pull/112
#define RCUTILS_BREAK std::abort
#endif
// Uncomment below intead
//#include <rcutils/assert.h>

namespace message_filters
{
namespace sync_policies
{

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct LatestPolicy : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<LatestPolicy> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;
  typedef std::deque<M0Event> M0Deque;
  typedef std::deque<M1Event> M1Deque;
  typedef std::deque<M2Event> M2Deque;
  typedef std::deque<M3Event> M3Deque;
  typedef std::deque<M4Event> M4Deque;
  typedef std::deque<M5Event> M5Deque;
  typedef std::deque<M6Event> M6Deque;
  typedef std::deque<M7Event> M7Deque;
  typedef std::deque<M8Event> M8Deque;
  typedef std::vector<M0Event> M0Vector;
  typedef std::vector<M1Event> M1Vector;
  typedef std::vector<M2Event> M2Vector;
  typedef std::vector<M3Event> M3Vector;
  typedef std::vector<M4Event> M4Vector;
  typedef std::vector<M5Event> M5Vector;
  typedef std::vector<M6Event> M6Vector;
  typedef std::vector<M7Event> M7Vector;
  typedef std::vector<M8Event> M8Vector;
  typedef Events Tuple;
  typedef std::tuple<M0Deque, M1Deque, M2Deque, M3Deque, M4Deque, M5Deque, M6Deque, M7Deque, M8Deque> DequeTuple;
  typedef std::tuple<M0Vector, M1Vector, M2Vector, M3Vector, M4Vector, M5Vector, M6Vector, M7Vector, M8Vector> VectorTuple;

  LatestPolicy()
  : parent_(0)
  , num_non_empty_deques_(0)
  {}

  LatestPolicy(const LatestPolicy& e)
  {
    *this = e;
  }

  LatestPolicy& operator=(const LatestPolicy& rhs)
  {
    parent_ = rhs.parent_;
    num_non_empty_deques_ = rhs.num_non_empty_deques_;

    deques_ = rhs.deques_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type& evt)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    deque.push_back(evt);

    if(i == (int)0)
    {
      RCUTILS_ASSERT(deque.size() == (size_t)1);// queue 0 should have only one the new added msg
      num_non_empty_deques_++;
      if(num_non_empty_deques_ == (uint32_t)RealTypeCount::value) // all queues have at least one message
      {
        // generate and publish one message group
        makeCandidate();
        publishCandidate();
      }
      else
      {
        deque.pop_front();
        --num_non_empty_deques_;
      }
    }
    else // we just add new message to the corresponding queue
    {
      if (deque.size() == (size_t)1) 
      {
        // We have just added the first message, so it was empty before
        ++num_non_empty_deques_;
      }
      else
      {
        deque.pop_front();
      }
    }
  }

private:
  void makeCandidate()
  {
    // Create candidate tuple
    candidate_ = Tuple(); // Discards old one if any
    std::get<0>(candidate_) = std::get<0>(deques_).back();
    std::get<1>(candidate_) = std::get<1>(deques_).back();
    if (RealTypeCount::value > 2)
    {
      std::get<2>(candidate_) = std::get<2>(deques_).back();
      if (RealTypeCount::value > 3)
      {
        std::get<3>(candidate_) = std::get<3>(deques_).back();
        if (RealTypeCount::value > 4)
        {
          std::get<4>(candidate_) = std::get<4>(deques_).back();
          if (RealTypeCount::value > 5)
          {
            std::get<5>(candidate_) = std::get<5>(deques_).back();
            if (RealTypeCount::value > 6)
            {
              std::get<6>(candidate_) = std::get<6>(deques_).back();
              if (RealTypeCount::value > 7)
              {
                std::get<7>(candidate_) = std::get<7>(deques_).back();
                if (RealTypeCount::value > 8)
                {
                  std::get<8>(candidate_) = std::get<8>(deques_).back();
                }
              }
            }
          }
        }
      }
    }

    std::get<0>(deques_).pop_front();
    RCUTILS_ASSERT(std::get<0>(deques_).size() == (size_t)0);
    --num_non_empty_deques_;
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  void publishCandidate()
  {
    // Publish
    RCUTILS_ASSERT(num_non_empty_deques_ == (uint32_t)RealTypeCount::value - (uint32_t)1);
    parent_->signal(std::get<0>(candidate_), std::get<1>(candidate_), std::get<2>(candidate_), std::get<3>(candidate_),
                    std::get<4>(candidate_), std::get<5>(candidate_), std::get<6>(candidate_), std::get<7>(candidate_),
                    std::get<8>(candidate_));
    // Delete this candidate
    candidate_ = Tuple();
  }

  Sync* parent_;
  
  DequeTuple deques_;
  uint32_t num_non_empty_deques_;
  Tuple candidate_;  // NULL if there is no candidate, in which case there is no pivot.
  std::mutex data_mutex_;  // Protects all of the above
};

}  // namespace sync
}  // namespace message_filters

#endif // LATEST_POLICY_H_
