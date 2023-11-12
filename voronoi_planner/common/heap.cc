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

#include "voronoi_planner/common/heap.h"

#include "glog/logging.h"

namespace voronoi_planner {
namespace common {

Heap::Heap() { queue_.resize(capacity_); }

Heap::Heap(const std::size_t capacity) : capacity_(capacity) {
  queue_.resize(capacity);
}

Heap::~Heap() { Clear(); }

void Heap::Clear() {
  for (std::size_t i = 1; i <= size_; ++i) {
    queue_[i].node->set_heap_index(0);
  }
  size_ = 0;
}

void Heap::PercolateDown(std::size_t hole, HeapElement obj) {
  // Sanity checks.
  CHECK(!Empty());

  std::size_t child;
  for (; 2 * hole <= size_; hole = child) {
    child = 2 * hole;
    if (child != size_ && queue_[child + 1].key < queue_[child].key) {
      ++child;
    }
    if (queue_[child].key < obj.key) {
      queue_[hole] = queue_[child];
      queue_[hole].node->set_heap_index(hole);
    } else {
      break;
    }
  }
  queue_[hole] = obj;
  queue_[hole].node->set_heap_index(hole);
}

void Heap::PercolateUp(std::size_t hole, HeapElement obj) {
  // Sanity checks.
  CHECK(!Empty());

  for (; hole > 1 && obj.key < queue_[hole / 2].key; hole /= 2) {
    queue_[hole] = queue_[hole / 2];
    queue_[hole].node->set_heap_index(hole);
  }
  queue_[hole] = obj;
  queue_[hole].node->set_heap_index(hole);
}

void Heap::PercolateUpOrDown(std::size_t hole, HeapElement obj) {
  // Sanity checks.
  CHECK(!Empty());

  if (hole > 1 && obj.key < queue_[hole / 2].key) {
    PercolateUp(hole, obj);
  } else {
    PercolateDown(hole, obj);
  }
}

void Heap::Allocate() {
  std::stringstream ss;
  ss << "Growing heap size from " << capacity_ << " to ";
  capacity_ *= 2;
  queue_.resize(capacity_);
  VLOG(4) << ss.str() << capacity_;
}

void Heap::Insert(Node* node, int key) {
  // Sanity checks.
  if (node->heap_index() != 0) {
    LOG(ERROR) << "The node is already in the heap";
    return;
  }

  if (size_ + 1 == capacity_) {
    Allocate();
  }

  HeapElement obj;
  obj.node = node;
  obj.key = key;
  ++size_;
  PercolateUp(size_, obj);
}

Node* Heap::Pop() {
  // Sanity checks.
  if (Empty()) {
    LOG(ERROR) << "The heap is empty";
    return nullptr;
  }

  Node* obj = queue_[1].node;
  obj->set_heap_index(0);
  if (size_ > 1) {
    PercolateDown(1, queue_[size_]);
  }
  --size_;
  return obj;
}

void Heap::Update(Node* node, int new_key) {
  // Sanity checks.
  if (node->heap_index() == 0) {
    LOG(ERROR) << "The node is not in the heap";
    return;
  }

  if (queue_[node->heap_index()].key != new_key) {
    queue_[node->heap_index()].key = new_key;
    PercolateUpOrDown(node->heap_index(), queue_[node->heap_index()]);
  }
}

}  // namespace common
}  // namespace voronoi_planner
