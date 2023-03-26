/*
 * Copyright (c) 2008, Maxim Likhachev
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
 *     * Neither the name of the Carnegie Mellon University nor the names of its
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

#include "voronoi_planner/heap.h"

#include "glog/logging.h"

namespace voronoi_planner {

Heap::Heap() { queue_.resize(capacity_); }

Heap::Heap(const size_t capacity) : capacity_(capacity) {
  queue_.resize(capacity);
}

Heap::~Heap() { Clear(); }

void Heap::Clear() {
  for (size_t i = 1; i <= size_; i++) {
    queue_[i].element->set_index(0);
  }
  size_ = 0;
}

void Heap::PercolateDown(size_t hole, HeapElement obj) {
  if (Empty()) {
    return;
  }

  size_t child;
  for (; 2 * hole <= size_; hole = child) {
    child = 2 * hole;
    if (child != size_ && queue_[child + 1].key < queue_[child].key) {
      child++;
    }
    if (queue_[child].key < obj.key) {
      queue_[hole] = queue_[child];
      queue_[hole].element->set_index(hole);
    } else {
      break;
    }
  }
  queue_[hole] = obj;
  queue_[hole].element->set_index(hole);
}

void Heap::PercolateUp(size_t hole, HeapElement obj) {
  if (Empty()) {
    return;
  }

  for (; hole > 1 && obj.key < queue_[hole / 2].key; hole /= 2) {
    queue_[hole] = queue_[hole / 2];
    queue_[hole].element->set_index(hole);
  }
  queue_[hole] = obj;
  queue_[hole].element->set_index(hole);
}

void Heap::PercolateUpOrDown(size_t hole, HeapElement obj) {
  if (Empty()) {
    return;
  }

  if (hole > 1 && obj.key < queue_[hole / 2].key) {
    PercolateUp(hole, obj);
  } else {
    PercolateDown(hole, obj);
  }
}

bool Heap::CheckSize() {
  if (size_ + 1 == HEAPSIZE) {
    LOG(ERROR) << "The heap is full";
    return false;
  }
  if (size_ + 1 == capacity_) {
    Allocate();
  }
  return true;
}

void Heap::Allocate() {
  std::stringstream ss;
  ss << "growing heap size from " << capacity_ << " to ";
  capacity_ = std::min(capacity_ * 2, HEAPSIZE);

  VLOG(4) << ss.str() << capacity_;
  queue_.resize(capacity_);
}

void Heap::Insert(SearchStateBase* search_state, int key) {
  CHECK_EQ(search_state->index(), 0);
  if (!CheckSize()) {
    return;
  }

  HeapElement obj;
  obj.element = search_state;
  obj.key = key;
  PercolateUp(++size_, obj);
}

int Heap::GetMinKey() const {
  if (Empty()) {
    return INFINITECOST;
  }
  return queue_[1].key;
}

SearchStateBase* Heap::Pop() {
  CHECK(!Empty());

  SearchStateBase* obj = queue_[1].element;
  obj->set_index(0);
  PercolateDown(1, queue_[size_--]);
  return obj;
}

void Heap::Update(SearchStateBase* search_state, int new_key) {
  CHECK_NE(search_state->index(), 0);

  if (queue_[search_state->index()].key != new_key) {
    queue_[search_state->index()].key = new_key;
    PercolateUpOrDown(search_state->index(), queue_[search_state->index()]);
  }
}

}  // namespace voronoi_planner
