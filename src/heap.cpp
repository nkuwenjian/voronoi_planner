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

#include <stdexcept>
#include "voronoi_planner/heap.h"

namespace voronoi_planner
{
void heaperror(const char* ErrorString)
{
  // need to send a message from here somehow
  throw std::runtime_error(ErrorString);
}

// constructors and destructors
CIntHeap::CIntHeap()
{
  percolates = 0;
  currentsize = 0;
  allocated = HEAPSIZE_INIT;

  heap = new heapintelement[allocated];
}

CIntHeap::CIntHeap(int initial_size)
{
  percolates = 0;
  currentsize = 0;
  allocated = initial_size;

  heap = new heapintelement[allocated];
}

CIntHeap::~CIntHeap()
{
  int i;
  for (i = 1; i <= currentsize; ++i)
    heap[i].heapstate->heapindex = 0;

  delete[] heap;
}

void CIntHeap::percolatedown(int hole, heapintelement tmp)
{
  int child;

  if (currentsize != 0)
  {
    for (; 2 * hole <= currentsize; hole = child)
    {
      child = 2 * hole;

      if (child != currentsize && heap[child + 1].key < heap[child].key)
        ++child;
      if (heap[child].key < tmp.key)
      {
        percolates += 1;
        heap[hole] = heap[child];
        heap[hole].heapstate->heapindex = hole;
      }
      else
        break;
    }
    heap[hole] = tmp;
    heap[hole].heapstate->heapindex = hole;
  }
}

void CIntHeap::percolateup(int hole, heapintelement tmp)
{
  if (currentsize != 0)
  {
    for (; hole > 1 && tmp.key < heap[hole / 2].key; hole /= 2)
    {
      percolates += 1;
      heap[hole] = heap[hole / 2];
      heap[hole].heapstate->heapindex = hole;
    }
    heap[hole] = tmp;
    heap[hole].heapstate->heapindex = hole;
  }
}

void CIntHeap::percolateupordown(int hole, heapintelement tmp)
{
  if (currentsize != 0)
  {
    if (hole > 1 && heap[hole / 2].key > tmp.key)
      percolateup(hole, tmp);
    else
      percolatedown(hole, tmp);
  }
}

bool CIntHeap::emptyheap()
{
  return currentsize == 0;
}

bool CIntHeap::fullheap()
{
  return currentsize == HEAPSIZE - 1;
}

bool CIntHeap::inheap(AbstractSearchState* AbstractSearchState)
{
  return (AbstractSearchState->heapindex != 0);
}

int CIntHeap::getkeyheap(AbstractSearchState* AbstractSearchState)
{
  if (AbstractSearchState->heapindex == 0)
    heaperror("GetKey: AbstractSearchState is not in heap");

  return heap[AbstractSearchState->heapindex].key;
}

void CIntHeap::makeemptyheap()
{
  int i;

  for (i = 1; i <= currentsize; ++i)
    heap[i].heapstate->heapindex = 0;
  currentsize = 0;
}

void CIntHeap::makeheap()
{
  int i;

  for (i = currentsize / 2; i > 0; i--)
  {
    percolatedown(i, heap[i]);
  }
}

void CIntHeap::growheap()
{
  heapintelement* newheap;
  int i;

  printf("growing heap size from %d ", allocated);

  allocated = 2 * allocated;
  if (allocated > HEAPSIZE)
    allocated = HEAPSIZE;

  printf("to %d\n", allocated);

  newheap = new heapintelement[allocated];

  for (i = 0; i <= currentsize; ++i)
    newheap[i] = heap[i];

  delete[] heap;

  heap = newheap;
}

void CIntHeap::sizecheck()
{
  if (fullheap())
    heaperror("insertheap: heap is full");
  else if (currentsize == allocated - 1)
  {
    growheap();
  }
}

void CIntHeap::insertheap(AbstractSearchState* AbstractSearchState, int key)
{
  heapintelement tmp;
  char strTemp[100];

  sizecheck();

  if (AbstractSearchState->heapindex != 0)
  {
    sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
    heaperror(strTemp);
  }
  tmp.heapstate = AbstractSearchState;
  tmp.key = key;
  percolateup(++currentsize, tmp);
}

void CIntHeap::deleteheap(AbstractSearchState* AbstractSearchState)
{
  if (AbstractSearchState->heapindex == 0)
    heaperror("deleteheap: AbstractSearchState is not in heap");
  percolateupordown(AbstractSearchState->heapindex, heap[currentsize--]);
  AbstractSearchState->heapindex = 0;
}

void CIntHeap::updateheap(AbstractSearchState* AbstractSearchState, int NewKey)
{
  if (AbstractSearchState->heapindex == 0)
    heaperror("Updateheap: AbstractSearchState is not in heap");
  if (heap[AbstractSearchState->heapindex].key != NewKey)
  {
    heap[AbstractSearchState->heapindex].key = NewKey;
    percolateupordown(AbstractSearchState->heapindex, heap[AbstractSearchState->heapindex]);
  }
}

AbstractSearchState* CIntHeap::getminheap()
{
  if (currentsize == 0)
    heaperror("GetMinheap: heap is empty");
  return heap[1].heapstate;
}

AbstractSearchState* CIntHeap::getminheap(int& ReturnKey)
{
  if (currentsize == 0)
  {
    heaperror("GetMinheap: heap is empty");
  }
  ReturnKey = heap[1].key;
  return heap[1].heapstate;
}

int CIntHeap::getminkeyheap()
{
  int ReturnKey;
  if (currentsize == 0)
    return INFINITECOST;
  ReturnKey = heap[1].key;
  return ReturnKey;
}

AbstractSearchState* CIntHeap::deleteminheap()
{
  AbstractSearchState* AbstractSearchState;

  if (currentsize == 0)
    heaperror("DeleteMin: heap is empty");

  AbstractSearchState = heap[1].heapstate;
  AbstractSearchState->heapindex = 0;
  percolatedown(1, heap[currentsize--]);
  return AbstractSearchState;
}

}  // namespace voronoi_planner
