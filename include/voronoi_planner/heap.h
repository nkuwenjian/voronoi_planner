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

#pragma once

#include <vector>

const size_t HEAPSIZE = 20000000;
const size_t HEAPSIZE_INIT = 5000;
#define INFINITECOST 1000000000

namespace voronoi_planner {
class SearchStateBase {
 public:
  SearchStateBase() = default;
  virtual ~SearchStateBase() = default;

  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

  size_t index_ = 0;
};

struct HeapElement {
  SearchStateBase* element = nullptr;
  int key = 0;
};

class Heap {
 public:
  Heap();
  explicit Heap(size_t capacity);
  virtual ~Heap();

  size_t Size() const { return size_; }
  bool Empty() const { return size_ == 0; }
  void Clear();
  void Insert(SearchStateBase* search_state, int key);
  void Update(SearchStateBase* search_state, int new_key);
  int GetMinKey() const;
  SearchStateBase* Pop();

 private:
  void PercolateUp(size_t hole, HeapElement obj);
  void PercolateDown(size_t hole, HeapElement obj);
  void PercolateUpOrDown(size_t hole, HeapElement obj);
  bool CheckSize();
  void Allocate();

 public:
  size_t size_ = 0;
  size_t capacity_ = HEAPSIZE_INIT;
  std::vector<HeapElement> queue_;
};

}  // namespace voronoi_planner
