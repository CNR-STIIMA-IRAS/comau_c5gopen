/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#ifndef __REALTIME_BUFFER_UTILS__
#define __REALTIME_BUFFER_UTILS__

#include <memory>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread/condition.hpp>

namespace realtime_buffer
{
// Thread safe circular buffer 
template <typename T>
class CircBuffer : private boost::noncopyable
{
public:
    typedef boost::mutex::scoped_lock lock;
    
    CircBuffer() {};
    ~CircBuffer() {}
    CircBuffer(int n) {cb_.set_capacity(n);}
    
    void push_back (T imdata) 
    {
      lock lk(monitor_);
      cb_.push_back(imdata);
      buffer_not_empty_.notify_one();
    }

    const T& front() 
    {
      lock lk(monitor_);
      while (cb_.empty())
          buffer_not_empty_.wait(lk);
      return cb_.front();
    }

    void pop_front() 
    {
      lock lk(monitor_);
      if(cb_.empty())
          return;
      return cb_.pop_front();
    }

    void clear() 
    {
        lock lk(monitor_);
        cb_.clear();
    }

    int size() 
    {
        lock lk(monitor_);
        return cb_.size();
    }

    void set_capacity(int capacity) 
    {
        lock lk(monitor_);
        cb_.set_capacity(capacity);
    }

    bool empty() 
    {
        lock lk(monitor_);
        return cb_.empty();
    }

    bool full() 
    {
        lock lk(monitor_);
        return cb_.full();
    }

    boost::circular_buffer<T>& get() 
    {
      return cb_;
    }

private:
    boost::condition buffer_not_empty_;
    boost::mutex monitor_;
    boost::circular_buffer<T> cb_;

};
template<typename T>
using CircBufferUnqPtr = std::unique_ptr<realtime_buffer::CircBuffer<T>>;
template<typename T>
using CircBufferShrPtr = std::shared_ptr<realtime_buffer::CircBuffer<T>>;
  

template< class K, class V >
std::map< K, V > combine( const std::vector< K >& keys, const std::vector< V >& values )
{
  assert( keys.size() == values.size() );
  std::map< K, V > ret;
  for( size_t i=0; i< keys.size(); i++) 
  {
    ret[ keys[i] ] = values[ i ];
  }
  return ret;
}

template <class T> class vector
{
private:
    std::vector<T>          standard_vector;
    mutable boost::mutex    mutex;

public:
    typedef typename std::vector<T>::iterator  iterator;
    typedef typename std::vector<T>::size_type size_type;

    explicit vector (const std::allocator<T>& alloc = std::allocator<T>()) : standard_vector( alloc ) {}
    explicit vector (size_type n)                                           : standard_vector( n ) {}
    vector (size_type n, const T& val, const std::allocator<T>& alloc = std::allocator<T>()) : standard_vector( n, val, alloc ) {}

    template <class InputIterator>
    vector (InputIterator first, InputIterator last, const std::allocator<T>& alloc = std::allocator<T>()) : standard_vector( first, last, alloc ) {};
    vector (const vector& x) : standard_vector(x){}
    vector (const vector& x, const std::allocator<T>& alloc) : standard_vector(x, alloc){}
    vector (vector&& x) : standard_vector( x ){}
    vector (vector&& x, const std::allocator<T>& alloc) : standard_vector(x, alloc) {}
    vector (std::initializer_list< T > il, const std::allocator<T>& alloc = std::allocator<T>()) : standard_vector(il, alloc) {}

    iterator begin(void)
    {
        boost::mutex::scoped_lock lock(mutex);
        return standard_vector.begin();
    }

    iterator end(void)
    {
        boost::mutex::scoped_lock lock(mutex);
        return standard_vector.end();
    }

    void push_back(T & item)
    {
        boost::mutex::scoped_lock lock(mutex);
        standard_vector.push_back(item);
    }
    void push_back(const T & item)
    {
        boost::mutex::scoped_lock lock(mutex);
        standard_vector.push_back(item);
    }


    void erase(iterator it)
    {
        boost::mutex::scoped_lock lock(mutex);
        standard_vector.erase(it);
    }
    std::vector<T>& get( ){ return standard_vector; }
    
    
};

};

#endif
