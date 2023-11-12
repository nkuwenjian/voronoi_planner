/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#pragma once

#include <vector>

#include "voronoi_planner/common/constants.h"
#include "voronoi_planner/common/node.h"

namespace voronoi_planner {
namespace common {

struct HeapElement {
  Node* node = nullptr;
  int key = 0;
};

class Heap {
 public:
  Heap();
  explicit Heap(std::size_t capacity);
  virtual ~Heap();

  bool Empty() const { return size_ == 0; }
  std::size_t Size() const { return size_; }
  void Clear();
  void Insert(Node* node, int key);
  void Update(Node* node, int new_key);
  int GetMinKey() const { return Empty() ? kInfiniteCost : queue_[1].key; }
  Node* Pop();

 private:
  void PercolateUp(std::size_t hole, HeapElement obj);
  void PercolateDown(std::size_t hole, HeapElement obj);
  void PercolateUpOrDown(std::size_t hole, HeapElement obj);
  void Allocate();

  std::size_t size_ = 0;
  std::size_t capacity_ = kInitHeapCapacity;
  std::vector<HeapElement> queue_;
};

}  // namespace common
}  // namespace voronoi_planner
